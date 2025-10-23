#include "ups_hid.h"
#include <cstring>

namespace esphome {
namespace ups_hid {

static const char *const TAG = "ups_hid";

// Flags estáticos
volatile bool UpsHid::g_probe_pending_   = false;
volatile bool UpsHid::g_polling_enabled_ = false;

// -----------------------------------------------------
// Pequeña utilidad para callbacks de control (no-op)
static void ctrl_transfer_cb_(usb_transfer_t *x) { (void)x; }

// log chunk en hex (corto)
void UpsHid::log_hex_prefix_(const char *prefix, const uint8_t *d, int n, int max_bytes) {
  if (n <= 0) {
    ESP_LOGI(TAG, "%s len=0", prefix);
    return;
  }
  if (max_bytes <= 0) max_bytes = n;
  if (max_bytes > n) max_bytes = n;

  char line[3 * 64 + 16];
  int k = 0;
  for (int i = 0; i < max_bytes; i++) {
    if (k + 4 >= (int)sizeof(line)) break;
    k += snprintf(line + k, sizeof(line) - k, "%02X%s", d[i], (i + 1 < max_bytes ? " " : ""));
  }
  ESP_LOGI(TAG, "%s len=%d data=%s%s", prefix, n, line, (n > max_bytes ? " ..." : ""));
}

// -----------------------------------------------------
// Lee el descriptor de configuración y localiza HID IF + EP IN
// -----------------------------------------------------
bool UpsHid::read_config_descriptor_and_log_hid_(usb_host_client_handle_t client,
                                                 usb_device_handle_t dev_handle,
                                                 uint8_t &if_num, uint8_t &ep_in,
                                                 uint16_t &mps, uint8_t &interval,
                                                 uint16_t &total_cfg_len) {
  if_num   = 0xFF;
  ep_in    = 0;
  mps      = 0;
  interval = 0;
  total_cfg_len = 0;

  if (!client || !dev_handle) return false;

  // 1) Header (9 bytes)
  const int hdr_len = 9;
  const int tot_hdr = USB_SETUP_PACKET_SIZE + hdr_len;

  usb_transfer_t *xhdr = nullptr;
  if (usb_host_transfer_alloc(tot_hdr, 0, &xhdr) != ESP_OK) {
    ESP_LOGW(TAG, "[cfg] transfer_alloc header failed");
    return false;
  }
  {
    auto *s = reinterpret_cast<usb_setup_packet_t *>(xhdr->data_buffer);
    s->bmRequestType = 0x80; // IN, device
    s->bRequest      = 0x06; // GET_DESCRIPTOR
    s->wValue        = (uint16_t)((2 << 8) | 0); // CONFIGURATION 0
    s->wIndex        = 0;
    s->wLength       = hdr_len;

    xhdr->num_bytes        = tot_hdr;
    xhdr->callback         = ctrl_transfer_cb_;
    xhdr->context          = nullptr;
    xhdr->device_handle    = dev_handle;
    xhdr->bEndpointAddress = 0x00;
    xhdr->flags            = 0;
  }
  if (usb_host_transfer_submit_control(client, xhdr) != ESP_OK) {
    ESP_LOGW(TAG, "[cfg] submit header failed");
    usb_host_transfer_free(xhdr);
    return false;
  }
  {
    TickType_t dl = xTaskGetTickCount() + pdMS_TO_TICKS(1500);
    while (xTaskGetTickCount() < dl) {
      (void) usb_host_client_handle_events(client, pdMS_TO_TICKS(10));
      if (xhdr->status != USB_TRANSFER_STATUS_IDLE) break;
    }
  }
  if (xhdr->status != USB_TRANSFER_STATUS_COMPLETED) {
    ESP_LOGW(TAG, "[cfg] header status=0x%X", (unsigned) xhdr->status);
    usb_host_transfer_free(xhdr);
    return false;
  }

  const uint8_t *cfg_hdr = xhdr->data_buffer + USB_SETUP_PACKET_SIZE;
  if (cfg_hdr[1] != 2 || cfg_hdr[0] < 9) {
    ESP_LOGW(TAG, "[cfg] invalid header bType=%u bLen=%u", cfg_hdr[1], cfg_hdr[0]);
    usb_host_transfer_free(xhdr);
    return false;
  }
  uint16_t wTotalLength = (uint16_t)(cfg_hdr[2] | (cfg_hdr[3] << 8));
  usb_host_transfer_free(xhdr);

  // 2) Descriptor completo
  total_cfg_len = wTotalLength;
  int payload = wTotalLength;
  int total   = USB_SETUP_PACKET_SIZE + payload;

  usb_transfer_t *xf = nullptr;
  if (usb_host_transfer_alloc(total, 0, &xf) != ESP_OK) {
    ESP_LOGW(TAG, "[cfg] transfer_alloc full failed");
    return false;
  }
  {
    auto *s = reinterpret_cast<usb_setup_packet_t *>(xf->data_buffer);
    s->bmRequestType = 0x80;
    s->bRequest      = 0x06; // GET_DESCRIPTOR
    s->wValue        = (uint16_t)((2 << 8) | 0);
    s->wIndex        = 0;
    s->wLength       = payload;

    xf->num_bytes        = total;
    xf->callback         = ctrl_transfer_cb_;
    xf->context          = nullptr;
    xf->device_handle    = dev_handle;
    xf->bEndpointAddress = 0x00;
    xf->flags            = 0;
  }
  if (usb_host_transfer_submit_control(client, xf) != ESP_OK) {
    ESP_LOGW(TAG, "[cfg] submit full failed");
    usb_host_transfer_free(xf);
    return false;
  }
  {
    TickType_t dl = xTaskGetTickCount() + pdMS_TO_TICKS(2000);
    while (xTaskGetTickCount() < dl) {
      (void) usb_host_client_handle_events(client, pdMS_TO_TICKS(10));
      if (xf->status != USB_TRANSFER_STATUS_IDLE) break;
    }
  }
  if (xf->status != USB_TRANSFER_STATUS_COMPLETED) {
    ESP_LOGW(TAG, "[cfg] full status=0x%X", (unsigned) xf->status);
    usb_host_transfer_free(xf);
    return false;
  }

  // 3) Parseo de IF/EP
  const uint8_t *p   = xf->data_buffer + USB_SETUP_PACKET_SIZE;
  const uint8_t *end = p + payload;

  int hid_if = -1;
  uint8_t found_ep_in = 0;
  uint16_t found_mps = 0;
  uint8_t found_itv = 0;

  while (p + 2 <= end && p[0] >= 2 && p + p[0] <= end) {
    uint8_t len = p[0], type = p[1];
    if (type == 4 && len >= 9) {  // INTERFACE
      uint8_t bInterfaceNumber = p[2];
      uint8_t bClass = p[5], bSub = p[6], bProto = p[7];
      if (bClass == 0x03 && hid_if < 0) {
        hid_if = bInterfaceNumber;
        ESP_LOGI(TAG, "[cfg] HID IF=%d class=0x%02X sub=0x%02X proto=0x%02X",
                 (int) bInterfaceNumber, bClass, bSub, bProto);
      }
    } else if (type == 5 && len >= 7 && hid_if >= 0 && found_ep_in == 0) { // ENDPOINT
      uint8_t bEndpointAddress = p[2];
      bool is_in   = (bEndpointAddress & 0x80) != 0;
      bool is_intr = ((p[3] & 0x03) == 3);
      if (is_in && is_intr) {
        found_ep_in = bEndpointAddress;
        found_mps   = (uint16_t)(p[4] | (p[5] << 8));
        found_itv   = p[6];
      }
    }
    p += len;
  }

  if (hid_if >= 0 && found_ep_in != 0) {
    if_num   = (uint8_t) hid_if;
    ep_in    = found_ep_in;
    mps      = found_mps;
    interval = found_itv;
    ESP_LOGI(TAG, "[cfg] HID endpoint IN=0x%02X MPS=%u interval=%u ms",
             ep_in, (unsigned) mps, (unsigned) interval);
    usb_host_transfer_free(xf);
    return true;
  }

  ESP_LOGW(TAG, "[cfg] No HID IF o EP IN encontrado");
  usb_host_transfer_free(xf);
  return false;
}

// -----------------------------------------------------
// Dump del Report Descriptor (GET_DESCRIPTOR(HID_REPORT))
// -----------------------------------------------------
bool UpsHid::dump_report_descriptor_chunked_(usb_host_client_handle_t client,
                                             usb_device_handle_t dev_handle,
                                             uint8_t if_num,
                                             uint16_t &out_total_len) {
  out_total_len = 0;
  if (!client || !dev_handle) return false;

  // 1) Primero, obtener el tamaño del HID Report Descriptor desde el HID descriptor
  //    Simplificamos: intentamos leer 512 bytes directamente (muchos UPS HID caben).
  //    Si es más, lo troceamos igualmente.
  const int CHUNK = 64;
  const int MAX_LEN = 1024;
  uint8_t  tmp[MAX_LEN];
  int      got = 0;

  // Leemos por bloques con GET_DESCRIPTOR (tipo=0x22) sobre interface (wIndex=if_num)
  while (got < MAX_LEN) {
    int req_len = (MAX_LEN - got) > CHUNK ? CHUNK : (MAX_LEN - got);
    int total   = USB_SETUP_PACKET_SIZE + req_len;

    usb_transfer_t *x = nullptr;
    if (usb_host_transfer_alloc(total, 0, &x) != ESP_OK) {
      ESP_LOGW(TAG, "[rdesc] alloc failed");
      break;
    }
    auto *s = reinterpret_cast<usb_setup_packet_t *>(x->data_buffer);
    s->bmRequestType = 0x81;            // IN, standard, interface
    s->bRequest      = 0x06;            // GET_DESCRIPTOR
    s->wValue        = (uint16_t)((0x22 << 8) | 0x00); // REPORT descriptor
    s->wIndex        = if_num;
    s->wLength       = req_len;

    x->num_bytes        = total;
    x->callback         = ctrl_transfer_cb_;
    x->context          = nullptr;
    x->device_handle    = dev_handle;
    x->bEndpointAddress = 0x00;
    x->flags            = 0;

    if (usb_host_transfer_submit_control(client, x) != ESP_OK) {
      ESP_LOGW(TAG, "[rdesc] submit failed");
      usb_host_transfer_free(x);
      break;
    }
    {
      TickType_t dl = xTaskGetTickCount() + pdMS_TO_TICKS(1000);
      while (xTaskGetTickCount() < dl) {
        (void) usb_host_client_handle_events(client, pdMS_TO_TICKS(10));
        if (x->status != USB_TRANSFER_STATUS_IDLE) break;
      }
    }
    if (x->status != USB_TRANSFER_STATUS_COMPLETED) {
      usb_host_transfer_free(x);
      break;
    }

    int got_now = x->actual_num_bytes;
    if (got_now <= 0) {
      usb_host_transfer_free(x);
      break;
    }

    // Copia payload (después del setup)
    int payload = got_now > CHUNK ? CHUNK : got_now;
    std::memcpy(tmp + got, x->data_buffer + USB_SETUP_PACKET_SIZE, payload);
    got += payload;

    usb_host_transfer_free(x);

    if (payload < CHUNK) {
      // Se acabó
      break;
    }
  }

  if (got > 0) {
    out_total_len = (uint16_t) got;
    ESP_LOGI(TAG, "[rdesc] len=%d bytes", got);
    // Troceamos para log
    const int L = 16;
    for (int i = 0; i < got; i += L) {
      int n = (i + L <= got) ? L : (got - i);
      char line[3 * L + 8];
      int k = 0;
      for (int j = 0; j < n; j++) {
        if (k + 4 >= (int)sizeof(line)) break;
        k += snprintf(line + k, sizeof(line) - k, "%02X%s", tmp[i + j], (j + 1 < n ? " " : ""));
      }
      ESP_LOGI(TAG, "[rdesc] %s", line);
    }
    return true;
  }

  ESP_LOGW(TAG, "[rdesc] vacío o error");
  return false;
}

// -----------------------------------------------------
// GET_REPORT (Input) por control, report-id
// -----------------------------------------------------
bool UpsHid::hid_get_report_input_ctrl_(usb_host_client_handle_t client,
                                        usb_device_handle_t dev_handle,
                                        uint8_t report_id,
                                        uint8_t *out_buf, int out_buf_len,
                                        int &out_len,
                                        uint8_t interface_number) {
  out_len = 0;
  if (!client || !dev_handle || !out_buf || out_buf_len <= 0) return false;

  int req = out_buf_len;
  int total = USB_SETUP_PACKET_SIZE + req;

  usb_transfer_t *x = nullptr;
  if (usb_host_transfer_alloc(total, 0, &x) != ESP_OK) {
    ESP_LOGW(TAG, "[ctrl] alloc failed");
    return false;
  }
  auto *s = reinterpret_cast<usb_setup_packet_t *>(x->data_buffer);
  s->bmRequestType = 0xA1;  // IN, class, interface
  s->bRequest      = 0x01;  // GET_REPORT
  s->wValue        = (uint16_t)((1 << 8) | report_id); // 1=Input report, low=ID
  s->wIndex        = interface_number;
  s->wLength       = req;

  x->num_bytes        = total;
  x->callback         = ctrl_transfer_cb_;
  x->context          = nullptr;
  x->device_handle    = dev_handle;
  x->bEndpointAddress = 0x00;
  x->flags            = 0;

  if (usb_host_transfer_submit_control(client, x) != ESP_OK) {
    ESP_LOGW(TAG, "[ctrl] submit failed");
    usb_host_transfer_free(x);
    return false;
  }
  {
    TickType_t dl = xTaskGetTickCount() + pdMS_TO_TICKS(1000);
    while (xTaskGetTickCount() < dl) {
      (void) usb_host_client_handle_events(client, pdMS_TO_TICKS(10));
      if (x->status != USB_TRANSFER_STATUS_IDLE) break;
    }
  }
  if (x->status != USB_TRANSFER_STATUS_COMPLETED) {
    ESP_LOGW(TAG, "[ctrl] status=0x%X", (unsigned) x->status);
    usb_host_transfer_free(x);
    return false;
  }

  int got = x->actual_num_bytes;
  if (got <= 0) {
    usb_host_transfer_free(x);
    return false;
  }

  // Payload (tras setup)
  int payload = got > out_buf_len ? out_buf_len : got;
  std::memcpy(out_buf, x->data_buffer + USB_SETUP_PACKET_SIZE, payload);
  out_len = payload;

  usb_host_transfer_free(x);
  return true;
}

// -----------------------------------------------------
// Diff de reportes por ID
// -----------------------------------------------------
void UpsHid::log_diff_report_(uint8_t rep_id, const uint8_t *data, int len) {
  if (len <= 0) return;

  // Limitar logs de diff a ~500ms por cortesía
  uint32_t now_ms = (uint32_t) (esp_log_timestamp() & 0xFFFFFFFFu);
  const uint32_t DIFF_LOG_MIN_PERIOD_MS = 300; // ajustable
  if ((now_ms - this->last_diff_log_ms_) < DIFF_LOG_MIN_PERIOD_MS) return;
  this->last_diff_log_ms_ = now_ms;

  const int MAX_SHOW = 64; // no esperamos más de 64
  int changed = 0;

  auto print_changes = [&](const uint8_t *oldb, int oldn, bool &have_flag){
    // compara byte a byte y reporta cambios
    char line[160];
    int k = 0;
    for (int i = 0; i < len && i < MAX_SHOW; i++) {
      uint8_t prev = (have_flag && i < oldn) ? oldb[i] : 0xFF; // 0xFF -> “sin valor previo”
      if (!have_flag || prev != data[i]) {
        if (k < (int)sizeof(line)-1) {
          int wrote = snprintf(line + k, sizeof(line) - k, "%s[%02d]:%02X->%02X",
                               (changed==0 ? "" : " "),
                               i, (have_flag ? prev : 0xFF), data[i]);
          if (wrote > 0) k += wrote;
          changed++;
        }
      }
    }
    if (changed > 0) {
      ESP_LOGI(TAG, "[diff] rep=0x%02X %s", rep_id, line);
    }
  };

  switch (rep_id) {
    case 0x01: {
      print_changes(this->last_rep_01_, this->last_rep_01_len_, this->have_last_01_);
      std::memcpy(this->last_rep_01_, data, len);
      this->last_rep_01_len_ = len;
      this->have_last_01_ = true;
      break;
    }
    case 0x64: {
      print_changes(this->last_rep_64_, this->last_rep_64_len_, this->have_last_64_);
      std::memcpy(this->last_rep_64_, data, len);
      this->last_rep_64_len_ = len;
      this->have_last_64_ = true;
      break;
    }
    case 0x66: {
      print_changes(this->last_rep_66_, this->last_rep_66_len_, this->have_last_66_);
      std::memcpy(this->last_rep_66_, data, len);
      this->last_rep_66_len_ = len;
      this->have_last_66_ = true;
      break;
    }
    default: {
      // Para otros IDs, solo log corto
      log_hex_prefix_("[poll] (other id) GET_REPORT", data, len, 32);
      break;
    }
  }
}

// =====================================================
// Métodos de la clase
// =====================================================

void UpsHid::setup() {
  // 1) Instalar host
  usb_host_config_t cfg = {.skip_phy_setup = false, .intr_flags = 0};
  esp_err_t err = usb_host_install(&cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "usb_host_install() failed: 0x%X", (unsigned) err);
    return;
  }
  ESP_LOGI(TAG, "USB Host Library installed.");

  // 2) Daemon task
  xTaskCreatePinnedToCore(UpsHid::host_daemon_task_, "usbh_daemon",
                          4096, nullptr, 5, nullptr, tskNO_AFFINITY);

  // 3) Cliente async
  usb_host_client_config_t ccfg = {};
  ccfg.is_synchronous    = false;
  ccfg.max_num_event_msg = 8;
  ccfg.async.client_event_callback = UpsHid::client_callback_;
  ccfg.async.callback_arg = this;

  err = usb_host_client_register(&ccfg, &this->client_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "usb_host_client_register() failed: 0x%X", (unsigned) err);
    return;
  }
  ESP_LOGI(TAG, "USB Host client registered.");

  // 4) Task del cliente
  xTaskCreatePinnedToCore(UpsHid::client_task_, "usbh_client",
                          4096, this, 5, nullptr, tskNO_AFFINITY);
}

void UpsHid::dump_config() {
  ESP_LOGCONFIG(TAG, "UPS HID component ready.");
}

// =====================================================
// Tareas / Callback
// =====================================================

void UpsHid::host_daemon_task_(void *arg) {
  uint32_t flags = 0;
  while (true) {
    esp_err_t err = usb_host_lib_handle_events(pdMS_TO_TICKS(1000), &flags);
    if (err == ESP_OK) {
      if (flags) {
        ESP_LOGI(TAG, "[usbh_daemon] USB Host event flags: 0x%08X", (unsigned) flags);
        flags = 0;
      }
    } else if (err != ESP_ERR_TIMEOUT) {
      ESP_LOGW(TAG, "[usbh_daemon] handle_events err=0x%X", (unsigned) err);
    }
  }
}

void UpsHid::client_task_(void *arg) {
  auto *self = static_cast<UpsHid *>(arg);
  if (!self || !self->client_) {
    ESP_LOGE(TAG, "[usbh_client] No client handle.");
    vTaskDelete(nullptr);
    return;
  }

  uint8_t  if_num = 0xFF, ep = 0; uint16_t mps = 0; uint8_t itv = 0; uint16_t cfg_len = 0;

  while (true) {
    esp_err_t err = usb_host_client_handle_events(self->client_, pdMS_TO_TICKS(100));
    if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
      ESP_LOGW(TAG, "[usbh_client] handle_events err=0x%X", (unsigned) err);
    }

    if (UpsHid::g_probe_pending_ && self->dev_handle_ != nullptr) {
      if (read_config_descriptor_and_log_hid_(self->client_, self->dev_handle_,
                                              if_num, ep, mps, itv, cfg_len)) {
        self->hid_if_        = if_num;
        self->hid_ep_in_     = ep;
        self->hid_ep_mps_    = mps;
        self->hid_ep_interval_ = itv;
        ESP_LOGI(TAG, "[cfg] ready: IF=%u EP=0x%02X MPS=%u interval=%u",
                 (unsigned) self->hid_if_, (unsigned) self->hid_ep_in_,
                 (unsigned) self->hid_ep_mps_, (unsigned) self->hid_ep_interval_);

        // dump report descriptor (troceado)
        uint16_t rdlen = 0;
        (void) dump_report_descriptor_chunked_(self->client_, self->dev_handle_,
                                               self->hid_if_, rdlen);

        UpsHid::g_polling_enabled_ = true;  // habilitamos polling control GET_REPORT
      }
      UpsHid::g_probe_pending_ = false;
    }

    // Polling simple por control GET_REPORT de los IDs vistos en tus logs
    if (UpsHid::g_polling_enabled_ && self->dev_handle_ != nullptr && self->hid_if_ != 0xFF) {
      const uint8_t ids[] = {0x01, 0x64, 0x66};
      uint8_t buf[UpsHid::MAX_REPORT_LEN];
      int     got = 0;

      for (uint8_t rid : ids) {
        if (hid_get_report_input_ctrl_(self->client_, self->dev_handle_, rid,
                                       buf, sizeof(buf), got, self->hid_if_)) {
          // Primer byte suele ser ReportID; si no, lo tratamos igual
          if (got > 0) {
            // Log corta + diff
            // self->log_hex_prefix_("[poll] GET_REPORT", buf, got, 32);
            self->log_diff_report_(rid, buf, got);
          }
        }
        // spacing pequeño entre ids para no acribillar el bus
        vTaskDelay(pdMS_TO_TICKS(10));
      }
      // periodo total de ~1s entre rondas
      vTaskDelay(pdMS_TO_TICKS(900));
    }
  }
}

void UpsHid::client_callback_(const usb_host_client_event_msg_t *msg, void *arg) {
  auto *self = static_cast<UpsHid *>(arg);
  if (!self) return;

  switch (msg->event) {
    case USB_HOST_CLIENT_EVENT_NEW_DEV: {
      esp_err_t e = usb_host_device_open(self->client_, msg->new_dev.address, &self->dev_handle_);
      if (e == ESP_OK) {
        self->dev_addr_ = msg->new_dev.address;
        ESP_LOGI(TAG, "[attach] NEW_DEV addr=%u (opened)", (unsigned) self->dev_addr_);
        UpsHid::g_probe_pending_ = true;
      } else {
        ESP_LOGW(TAG, "[attach] NEW_DEV addr=%u but open failed: 0x%X",
                 (unsigned) msg->new_dev.address, (unsigned) e);
      }
      break;
    }
    case USB_HOST_CLIENT_EVENT_DEV_GONE: {
      UpsHid::g_polling_enabled_ = false;

      if (self->dev_handle_ != nullptr) {
        usb_host_device_close(self->client_, self->dev_handle_);
        self->dev_handle_ = nullptr;
      }

      self->dev_addr_ = 0;
      self->hid_if_   = 0xFF;
      self->hid_ep_in_ = 0;
      self->hid_ep_mps_ = 0;
      self->hid_ep_interval_ = 0;

      self->have_last_01_ = self->have_last_64_ = self->have_last_66_ = false;
      self->last_rep_01_len_ = self->last_rep_64_len_ = self->last_rep_66_len_ = 0;

      ESP_LOGI(TAG, "[detach] DEV_GONE");
      break;
    }
    default:
      ESP_LOGI(TAG, "[client] event=%d", (int) msg->event);
      break;
  }
}

}  // namespace ups_hid
}  // namespace esphome

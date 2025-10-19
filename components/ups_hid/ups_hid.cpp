#include "ups_hid.h"

namespace esphome {
namespace ups_hid {

static const char *const TAG = "ups_hid";

// ------------------------------------------------------------------
// Pequeña utilidad: callback "no-op" requerido por IDF en transfers
// ------------------------------------------------------------------
static void ctrl_transfer_cb_(usb_transfer_t *xfer) {
  (void) xfer;  // no hacemos nada; solo evitar "callback is NULL"
}

// ------------------------------------------------------------------
// En IDF 5.4.x hay que bombear eventos del cliente mientras esperamos
// una transferencia de control. Esta función espera con timeout.
// ------------------------------------------------------------------
static bool wait_ctrl_done_(usb_host_client_handle_t client, usb_transfer_t *xfer, uint32_t timeout_ms) {
  const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
  while (xTaskGetTickCount() < deadline) {
    (void) usb_host_client_handle_events(client, pdMS_TO_TICKS(10));
    if (xfer->status == USB_TRANSFER_STATUS_COMPLETED
        || xfer->status == USB_TRANSFER_STATUS_ERROR
        || xfer->status == USB_TRANSFER_STATUS_STALL
        || xfer->status == USB_TRANSFER_STATUS_NO_DEVICE
        || xfer->status == USB_TRANSFER_STATUS_CANCELED) {
      break;
    }
  }
  return xfer->status == USB_TRANSFER_STATUS_COMPLETED;
}

// ------------------------------------------------------------------
// Lee el Configuration Descriptor completo y localiza:
//  - primera INTERFACE de clase HID (0x03) => if_num
//  - su ENDPOINT IN interrupt => ep_in, mps, interval
// Devuelve true si lo encuentra (y rellena parámetros).
// ------------------------------------------------------------------
static bool read_config_descriptor_and_log_hid_(usb_host_client_handle_t client,
                                                usb_device_handle_t dev_handle,
                                                uint8_t &if_num, uint8_t &ep_in,
                                                uint16_t &mps, uint8_t &interval,
                                                uint16_t &total_len_out) {
  if_num = 0xFF; ep_in = 0; mps = 0; interval = 0; total_len_out = 0;
  if (!client || !dev_handle) return false;

  // A) Leer cabecera del Configuration Descriptor (9 bytes)
  const int hdr_len = 9;
  const int hdr_tot = USB_SETUP_PACKET_SIZE + hdr_len;
  usb_transfer_t *xhdr = nullptr;
  if (usb_host_transfer_alloc(hdr_tot, 0, &xhdr) != ESP_OK) {
    ESP_LOGW(TAG, "[cfg] transfer_alloc header failed");
    return false;
  }
  usb_setup_packet_t *sh = (usb_setup_packet_t *) xhdr->data_buffer;
  sh->bmRequestType = 0x80;                      // IN | Standard | Device
  sh->bRequest      = 0x06;                      // GET_DESCRIPTOR
  sh->wValue        = (uint16_t)((2 << 8) | 0);  // CONFIGURATION(2) << 8 | 0
  sh->wIndex        = 0;
  sh->wLength       = hdr_len;

  xhdr->num_bytes        = hdr_tot;
  xhdr->callback         = ctrl_transfer_cb_;
  xhdr->context          = nullptr;
  xhdr->device_handle    = dev_handle;
  xhdr->bEndpointAddress = 0x00;
  xhdr->flags            = 0;

  if (usb_host_transfer_submit_control(client, xhdr) != ESP_OK) {
    ESP_LOGW(TAG, "[cfg] submit header failed");
    usb_host_transfer_free(xhdr);
    return false;
  }
  if (!wait_ctrl_done_(client, xhdr, 1500)) {
    ESP_LOGW(TAG, "[cfg] header transfer failed status=0x%X", (unsigned) xhdr->status);
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
  total_len_out = wTotalLength;
  usb_host_transfer_free(xhdr);

  // B) Leer descriptor completo
  int payload = wTotalLength;
  if (payload < 9) payload = 9;  // por seguridad
  int total   = USB_SETUP_PACKET_SIZE + payload;
  usb_transfer_t *xfull = nullptr;
  if (usb_host_transfer_alloc(total, 0, &xfull) != ESP_OK) {
    ESP_LOGW(TAG, "[cfg] transfer_alloc full failed");
    return false;
  }
  usb_setup_packet_t *sf = (usb_setup_packet_t *) xfull->data_buffer;
  sf->bmRequestType = 0x80;
  sf->bRequest      = 0x06;
  sf->wValue        = (uint16_t)((2 << 8) | 0);
  sf->wIndex        = 0;
  sf->wLength       = payload;

  xfull->num_bytes        = total;
  xfull->callback         = ctrl_transfer_cb_;
  xfull->context          = nullptr;
  xfull->device_handle    = dev_handle;
  xfull->bEndpointAddress = 0x00;
  xfull->flags            = 0;

  if (usb_host_transfer_submit_control(client, xfull) != ESP_OK) {
    ESP_LOGW(TAG, "[cfg] submit full failed");
    usb_host_transfer_free(xfull);
    return false;
  }
  if (!wait_ctrl_done_(client, xfull, 2000)) {
    ESP_LOGW(TAG, "[cfg] full transfer failed status=0x%X", (unsigned) xfull->status);
    usb_host_transfer_free(xfull);
    return false;
  }

  // C) Parseo lineal
  const uint8_t *p   = xfull->data_buffer + USB_SETUP_PACKET_SIZE;
  const uint8_t *end = p + payload;
  int hid_if_seen = -1;
  while (p + 2 <= end && p[0] >= 2 && p + p[0] <= end) {
    uint8_t len = p[0], type = p[1];
    if (type == 4 && len >= 9) {  // INTERFACE
      uint8_t ifnum = p[2];
      uint8_t cls   = p[5], sub = p[6], proto = p[7];
      if (cls == 0x03 && hid_if_seen < 0) {
        hid_if_seen = ifnum;
        if_num = ifnum;
        ESP_LOGI(TAG, "[cfg] HID IF=%d class=0x%02X sub=0x%02X proto=0x%02X",
                 (int)ifnum, cls, sub, proto);
      }
    } else if (type == 5 && len >= 7 && hid_if_seen >= 0 && ep_in == 0) {  // ENDPOINT
      uint8_t bEndpointAddress = p[2];
      bool is_in   = (bEndpointAddress & 0x80) != 0;
      bool is_intr = ((p[3] & 0x03) == 3);
      if (is_in && is_intr) {
        ep_in   = bEndpointAddress;
        mps     = (uint16_t)(p[4] | (p[5] << 8));
        interval= p[6];
      }
    }
    p += len;
  }

  if (hid_if_seen >= 0 && ep_in != 0) {
    ESP_LOGI(TAG, "[cfg] HID endpoint IN=0x%02X MPS=%u interval=%u ms",
             ep_in, (unsigned)mps, (unsigned)interval);
    usb_host_transfer_free(xfull);
    return true;
  } else {
    ESP_LOGW(TAG, "[cfg] No se encontró interfaz HID o endpoint IN.");
    usb_host_transfer_free(xfull);
    return false;
  }
}

// ------------------------------------------------------------------
// GET_DESCRIPTOR (Report Descriptor) para una interface HID
// ------------------------------------------------------------------
static bool dump_report_descriptor_(usb_host_client_handle_t client,
                                    usb_device_handle_t dev_handle,
                                    uint8_t if_num,
                                    uint16_t max_len_hint) {
  if (!client || !dev_handle || if_num == 0xFF) return false;

  // Intentamos hasta 1024, pero empezamos con hint (de config wTotalLength no es el rdesc)
  uint16_t ask = 1024;
  if (max_len_hint >= 32 && max_len_hint <= 1024) ask = max_len_hint;

  int total = USB_SETUP_PACKET_SIZE + ask;
  usb_transfer_t *x = nullptr;
  if (usb_host_transfer_alloc(total, 0, &x) != ESP_OK) {
    ESP_LOGW(TAG, "[rdesc] alloc failed");
    return false;
  }
  usb_setup_packet_t *s = (usb_setup_packet_t *) x->data_buffer;
  s->bmRequestType = 0x81;                        // IN | Standard | Interface
  s->bRequest      = 0x06;                        // GET_DESCRIPTOR
  s->wValue        = (uint16_t)((0x22 << 8) | 0); // REPORT(0x22) << 8 | index 0
  s->wIndex        = if_num;
  s->wLength       = ask;

  x->num_bytes        = total;
  x->callback         = ctrl_transfer_cb_;
  x->context          = nullptr;
  x->device_handle    = dev_handle;
  x->bEndpointAddress = 0x00;
  x->flags            = 0;

  if (usb_host_transfer_submit_control(client, x) != ESP_OK) {
    ESP_LOGW(TAG, "[rdesc] submit failed");
    usb_host_transfer_free(x);
    return false;
  }
  if (!wait_ctrl_done_(client, x, 2000)) {
    ESP_LOGW(TAG, "[rdesc] transfer failed status=0x%X", (unsigned) x->status);
    usb_host_transfer_free(x);
    return false;
  }

  int got = x->actual_num_bytes - USB_SETUP_PACKET_SIZE;
  if (got < 0) got = 0;
  ESP_LOGI(TAG, "[rdesc] len=%d bytes", got);

  // Logueo por líneas de 16 bytes
  const uint8_t *d = x->data_buffer + USB_SETUP_PACKET_SIZE;
  for (int i = 0; i < got; i += 16) {
    char line[16 * 3 + 1];
    int  k = 0;
    for (int j = 0; j < 16 && (i + j) < got; j++) {
      k += snprintf(line + k, sizeof(line) - k, "%02X%s", d[i + j], (j == 15 || i + j + 1 == got) ? "" : " ");
      if (k >= (int)sizeof(line)) break;
    }
    ESP_LOGI(TAG, "[rdesc] %s", line);
  }

  usb_host_transfer_free(x);
  return true;
}

// ------------------------------------------------------------------
// HID GET_REPORT (Input) por control transfer (IDF 5.4.x)
// report_type: 1 = Input
// wValue: (report_type << 8) | report_id
// wIndex: interface number
// ------------------------------------------------------------------
static bool hid_get_report_input_ctrl_(usb_host_client_handle_t client,
                                       usb_device_handle_t dev_handle,
                                       uint8_t if_num,
                                       uint8_t report_id,
                                       uint8_t *out_buf,
                                       int out_buf_size,
                                       int &out_len) {
  out_len = 0;
  if (!client || !dev_handle || if_num == 0xFF || !out_buf || out_buf_size <= 0) return false;

  int total = USB_SETUP_PACKET_SIZE + out_buf_size;
  usb_transfer_t *x = nullptr;
  if (usb_host_transfer_alloc(total, 0, &x) != ESP_OK) return false;

  usb_setup_packet_t *s = (usb_setup_packet_t *) x->data_buffer;
  s->bmRequestType = 0xA1; // IN | Class | Interface
  s->bRequest      = 0x01; // GET_REPORT
  s->wValue        = (uint16_t)((0x01 << 8) | report_id); // Input report, ID
  s->wIndex        = if_num;
  s->wLength       = out_buf_size;

  x->num_bytes        = total;
  x->callback         = ctrl_transfer_cb_;
  x->context          = nullptr;
  x->device_handle    = dev_handle;
  x->bEndpointAddress = 0x00;
  x->flags            = 0;

  if (usb_host_transfer_submit_control(client, x) != ESP_OK) {
    usb_host_transfer_free(x);
    return false;
  }
  if (!wait_ctrl_done_(client, x, 1000)) {
    usb_host_transfer_free(x);
    return false;
  }

  int got = x->actual_num_bytes - USB_SETUP_PACKET_SIZE;
  if (got < 0) got = 0;
  if (got > out_buf_size) got = out_buf_size;

  const uint8_t *src = x->data_buffer + USB_SETUP_PACKET_SIZE;
  std::memcpy(out_buf, src, got);
  out_len = got;

  usb_host_transfer_free(x);
  return true;
}

// =====================================================
// Variables de coordinación fuera del callback
// =====================================================
static UpsHid *g_self = nullptr;
static volatile bool g_probe_pending = false;  // hacer lectura cfg+rdesc fuera del callback

// =====================================================
// Métodos de la clase
// =====================================================

void UpsHid::setup() {
  // 1) Instalar librería USB Host
  usb_host_config_t cfg = {.skip_phy_setup = false, .intr_flags = 0};
  esp_err_t err = usb_host_install(&cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "usb_host_install() failed: 0x%X", (unsigned) err);
    return;
  }
  ESP_LOGI(TAG, "USB Host Library installed.");

  // 2) Tarea daemon de la librería
  xTaskCreatePinnedToCore(UpsHid::host_daemon_task_, "usbh_daemon",
                          4096, nullptr, 5, nullptr, tskNO_AFFINITY);

  // 3) Registrar cliente asíncrono con callback
  usb_host_client_config_t client_cfg = {
      .is_synchronous = false,
      .max_num_event_msg = 8,
      .async = {
          .client_event_callback = UpsHid::client_callback_,
          .callback_arg = this,
      },
  };
  err = usb_host_client_register(&client_cfg, &this->client_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "usb_host_client_register() failed: 0x%X", (unsigned) err);
    return;
  }
  ESP_LOGI(TAG, "USB Host client registered.");

  // 4) Tarea que despacha eventos + sondeo periódico
  xTaskCreatePinnedToCore(UpsHid::client_task_, "usbh_client",
                          6144, this, 5, nullptr, tskNO_AFFINITY);

  g_self = this;
}

void UpsHid::dump_config() {
  ESP_LOGCONFIG(TAG, "UPS HID component is configured.");
  ESP_LOGI(TAG, "UPS Host init step ready (no HID yet).");
}

void UpsHid::update() {
  if (!this->hello_logged_) {
    ESP_LOGI(TAG, "UPS HID component started (hello from ESPHome external component).");
    this->hello_logged_ = true;
  }
}

// =====================================================
// Funciones estáticas (tareas/callback)
// =====================================================

void UpsHid::host_daemon_task_(void *arg) {
  (void) arg;
  uint32_t flags = 0;
  while (true) {
    esp_err_t err = usb_host_lib_handle_events(pdMS_TO_TICKS(1000), &flags);
    if (err == ESP_OK) {
      if (flags) {
        ESP_LOGI(TAG, "[usbh_daemon] USB Host event flags: 0x%08X", (unsigned) flags);
        flags = 0;
      }
    } else if (err == ESP_ERR_TIMEOUT) {
      // sin eventos
    } else {
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

  // ticks para sondeo
  self->next_poll_tick_ = xTaskGetTickCount() + pdMS_TO_TICKS(1000);

  while (true) {
    esp_err_t err = usb_host_client_handle_events(self->client_, pdMS_TO_TICKS(100));
    if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
      ESP_LOGW(TAG, "[usbh_client] handle_events err=0x%X", (unsigned) err);
    }

    // A) Descubrimiento (leer cfg + endpoint + rdesc) fuera del callback
    if (g_probe_pending && self->dev_handle_ != nullptr) {
      uint8_t  ifnum = 0xFF, ep = 0; uint16_t mps = 0; uint8_t itv = 0; uint16_t cfglen = 0;
      if (read_config_descriptor_and_log_hid_(self->client_, self->dev_handle_, ifnum, ep, mps, itv, cfglen)) {
        self->hid_if_         = ifnum;
        self->hid_ep_in_      = ep;
        self->hid_ep_mps_     = mps;
        self->hid_ep_interval_= itv;
        ESP_LOGI(TAG, "[cfg] ready: IF=%u EP=0x%02X MPS=%u interval=%u", (unsigned)ifnum, ep, (unsigned)mps, (unsigned)itv);

        // Volcar report descriptor (una vez)
        (void) dump_report_descriptor_(self->client_, self->dev_handle_, self->hid_if_, 512);

        // habilitar sondeo periódico por control
        self->poll_enabled_ = true;
        self->next_poll_tick_ = xTaskGetTickCount() + pdMS_TO_TICKS(1000);
      }
      g_probe_pending = false;
    }

    // B) Sondeo periódico GET_REPORT (Input) por control
    if (self->poll_enabled_
        && self->dev_handle_ != nullptr
        && self->hid_if_ != 0xFF
        && (int32_t)(xTaskGetTickCount() - self->next_poll_tick_) >= 0) {

      // Lista de report IDs a probar (ajustable)
      const uint8_t report_ids[] = {0x01, 0x64, 0x65, 0x66, 0x67};
      uint8_t buf[64];
      int len = 0;

      for (uint8_t rid : report_ids) {
        if (hid_get_report_input_ctrl_(self->client_, self->dev_handle_, self->hid_if_, rid, buf, sizeof(buf), len) && len > 0) {
          // Log breve: hasta 32 bytes
          int max_log = len < 64 ? len : 64;
          char line[64 * 3 + 1]; int k = 0;
          for (int i = 0; i < max_log; i++) {
            k += snprintf(line + k, sizeof(line) - k, "%02X%s", buf[i], (i + 1 < max_log ? " " : ""));
            if (k >= (int)sizeof(line)) break;
          }
          ESP_LOGI(TAG, "[poll] GET_REPORT id=0x%02X len=%d data=%s%s",
                   rid, len, line, (len > max_log ? " ..." : ""));
        }
      }

      self->next_poll_tick_ = xTaskGetTickCount() + pdMS_TO_TICKS(1000);  // cada 1s
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
        g_probe_pending = true;  // dispara lectura cfg+rdesc fuera del callback
      } else {
        ESP_LOGW(TAG, "[attach] NEW_DEV addr=%u but open failed: 0x%X",
                 (unsigned) msg->new_dev.address, (unsigned) e);
      }
      break;
    }

    case USB_HOST_CLIENT_EVENT_DEV_GONE: {
      self->poll_enabled_ = false;
      if (self->dev_handle_ != nullptr) {
        usb_host_device_close(self->client_, self->dev_handle_);
        self->dev_handle_ = nullptr;
      }
      self->dev_addr_        = 0;
      self->hid_if_          = 0xFF;
      self->hid_ep_in_       = 0;
      self->hid_ep_mps_      = 0;
      self->hid_ep_interval_ = 0;
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

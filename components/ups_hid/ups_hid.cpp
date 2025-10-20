#include "ups_hid.h"
#include "usb/usb_types_ch9.h"  // usb_setup_packet_t, USB_SETUP_PACKET_SIZE
#include "esp_timer.h"          // esp_timer_get_time

namespace esphome {
namespace ups_hid {

static const char *const TAG = "ups_hid";

// Estado global mínimo para tareas
static UpsHid *g_self = nullptr;

// ------------------------------------------
// Helpers de logging
// ------------------------------------------
void UpsHid::log_hex_line_(const char *prefix, const uint8_t *data, int len) {
  // imprime hasta 16 bytes por línea
  char buf[3 * 16 + 1];
  int k = 0;
  for (int i = 0; i < len && i < 16; i++) {
    k += snprintf(buf + k, sizeof(buf) - k, "%02X%s", data[i], (i + 1 < len && i + 1 < 16 ? " " : ""));
    if (k >= (int) sizeof(buf)) break;
  }
  ESP_LOGI(TAG, "%s %s", prefix, buf);
}

void UpsHid::store_last_report_(uint8_t report_id, const uint8_t *data, int len) {
  if (len <= 0) return;
  if (report_id == 0x01) {
    int idx = last_rep_id01_count_ % MAX_STORED_REPORT;
    memset(last_rep_id01_[idx], 0, sizeof(last_rep_id01_[idx]));
    memcpy(last_rep_id01_[idx], data, len > 64 ? 64 : len);
    last_rep_id01_count_++;
  } else if (report_id == 0x64) {
    int idx = last_rep_id64_count_ % MAX_STORED_REPORT;
    memset(last_rep_id64_[idx], 0, sizeof(last_rep_id64_[idx]));
    memcpy(last_rep_id64_[idx], data, len > 64 ? 64 : len);
    last_rep_id64_count_++;
  }
}

// ------------------------------------------
// Helper común de espera de control transfer
// Espera hasta COMPLETED o error conocido (compat IDF sin *_NOT_READY)
// ------------------------------------------
static bool wait_ctrl_done_(usb_host_client_handle_t client,
                            usb_transfer_t *xfer,
                            uint32_t timeout_ms) {
  const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
  while (xTaskGetTickCount() < deadline) {
    (void) usb_host_client_handle_events(client, pdMS_TO_TICKS(10));
    if (xfer->status == USB_TRANSFER_STATUS_COMPLETED ||
        xfer->status == USB_TRANSFER_STATUS_ERROR ||
        xfer->status == USB_TRANSFER_STATUS_STALL ||
        xfer->status == USB_TRANSFER_STATUS_NO_DEVICE ||
        xfer->status == USB_TRANSFER_STATUS_CANCELED) {
      return (xfer->status == USB_TRANSFER_STATUS_COMPLETED);
    }
  }
  // timeout: no cambió a un estado terminal conocido
  return false;
}

static void ctrl_noop_cb(usb_transfer_t *t) { (void)t; }

// ------------------------------------------
// Control transfers: Config Descriptor (HID IF/EP) y Report Descriptor
// ------------------------------------------
bool UpsHid::read_config_descriptor_and_log_hid_(
    usb_host_client_handle_t client,
    usb_device_handle_t dev_handle,
    uint8_t &if_num, uint8_t &ep_in,
    uint16_t &mps, uint8_t &interval,
    uint16_t &total_len) {

  if_num = 0xFF; ep_in = 0; mps = 0; interval = 0; total_len = 0;
  if (!client || !dev_handle) return false;

  // 1) Leer cabecera de configuración (9 bytes)
  const int hdr_len = 9;
  const int hdr_total = USB_SETUP_PACKET_SIZE + hdr_len;

  usb_transfer_t *xhdr = nullptr;
  if (usb_host_transfer_alloc(hdr_total, 0, &xhdr) != ESP_OK) {
    ESP_LOGW(TAG, "[cfg] alloc header failed");
    return false;
  }

  usb_setup_packet_t *sp = (usb_setup_packet_t *) xhdr->data_buffer;
  sp->bmRequestType = 0x80; // IN, Standard, Device
  sp->bRequest      = 0x06; // GET_DESCRIPTOR
  sp->wValue        = (uint16_t)((2 << 8) | 0); // CONFIG (2), index 0
  sp->wIndex        = 0;
  sp->wLength       = hdr_len;

  xhdr->num_bytes = hdr_total;
  xhdr->callback = ctrl_noop_cb;
  xhdr->context = nullptr;
  xhdr->device_handle = dev_handle;
  xhdr->bEndpointAddress = 0x00;
  xhdr->flags = 0;

  if (usb_host_transfer_submit_control(client, xhdr) != ESP_OK) {
    ESP_LOGW(TAG, "[cfg] submit header failed");
    usb_host_transfer_free(xhdr);
    return false;
  }

  bool ok_hdr = wait_ctrl_done_(client, xhdr, 1000);
  if (!ok_hdr) {
    ESP_LOGW(TAG, "[cfg] header status=0x%X", (unsigned) xhdr->status);
    usb_host_transfer_free(xhdr);
    return false;
  }

  const uint8_t *cfg_hdr = xhdr->data_buffer + USB_SETUP_PACKET_SIZE;
  if (cfg_hdr[1] != 2 || cfg_hdr[0] < 9) {
    ESP_LOGW(TAG, "[cfg] invalid header type/len");
    usb_host_transfer_free(xhdr);
    return false;
  }

  uint16_t wTotalLength = (uint16_t)(cfg_hdr[2] | (cfg_hdr[3] << 8));
  total_len = wTotalLength;
  usb_host_transfer_free(xhdr);

  // 2) Leer configuración completa
  int payload = wTotalLength;
  int total   = USB_SETUP_PACKET_SIZE + payload;
  usb_transfer_t *xf = nullptr;
  if (usb_host_transfer_alloc(total, 0, &xf) != ESP_OK) {
    ESP_LOGW(TAG, "[cfg] alloc full failed");
    return false;
  }

  usb_setup_packet_t *sp2 = (usb_setup_packet_t *) xf->data_buffer;
  sp2->bmRequestType = 0x80;
  sp2->bRequest      = 0x06;
  sp2->wValue        = (uint16_t)((2 << 8) | 0);
  sp2->wIndex        = 0;
  sp2->wLength       = payload;

  xf->num_bytes = total;
  xf->callback = ctrl_noop_cb;
  xf->context = nullptr;
  xf->device_handle = dev_handle;
  xf->bEndpointAddress = 0x00;
  xf->flags = 0;

  if (usb_host_transfer_submit_control(client, xf) != ESP_OK) {
    ESP_LOGW(TAG, "[cfg] submit full failed");
    usb_host_transfer_free(xf);
    return false;
  }

  bool ok_full = wait_ctrl_done_(client, xf, 1500);
  if (!ok_full) {
    ESP_LOGW(TAG, "[cfg] full status=0x%X", (unsigned) xf->status);
    usb_host_transfer_free(xf);
    return false;
  }

  // Parsear: INTERFACE HID + ENDPOINT IN interrupt
  const uint8_t *p = xf->data_buffer + USB_SETUP_PACKET_SIZE;
  const uint8_t *end = p + payload;

  int hid_if = -1;
  while (p + 2 <= end && p[0] >= 2 && p + p[0] <= end) {
    uint8_t len = p[0], type = p[1];

    if (type == 4 && len >= 9) { // INTERFACE
      uint8_t bInterfaceNumber = p[2];
      uint8_t bClass = p[5], bSub = p[6], bProto = p[7];
      if (bClass == 0x03 && hid_if < 0) {
        hid_if = bInterfaceNumber;
        ESP_LOGI(TAG, "[cfg] HID IF=%d class=0x%02X sub=0x%02X proto=0x%02X",
                 (int) bInterfaceNumber, bClass, bSub, bProto);
      }
    } else if (type == 5 && len >= 7 && hid_if >= 0 && ep_in == 0) { // ENDPOINT
      uint8_t bEndpointAddress = p[2];
      bool is_in = (bEndpointAddress & 0x80) != 0;
      bool is_intr = ((p[3] & 0x03) == 3);
      if (is_in && is_intr) {
        ep_in = bEndpointAddress;
        mps = (uint16_t)(p[4] | (p[5] << 8));
        interval = p[6];
      }
    }
    p += len;
  }

  usb_host_transfer_free(xf);

  if (hid_if >= 0 && ep_in != 0) {
    if_num = (uint8_t) hid_if;
    ESP_LOGI(TAG, "[cfg] HID endpoint IN=0x%02X MPS=%u interval=%u ms",
             ep_in, (unsigned) mps, (unsigned) interval);
    return true;
  } else {
    ESP_LOGW(TAG, "[cfg] No se encontró interfaz HID o endpoint IN.");
    return false;
  }
}

bool UpsHid::dump_report_descriptor_chunked_(
    usb_host_client_handle_t client,
    usb_device_handle_t dev_handle,
    uint8_t interface_number,
    uint16_t &out_len) {

  out_len = 0;
  if (!client || !dev_handle) return false;

  // GET_DESCRIPTOR (class/interface) -> Report Descriptor (0x22)
  int req = 512;  // tu UPS suele dar 512–604; si se quedara corto, podremos aumentar en una segunda pasada
  int total = USB_SETUP_PACKET_SIZE + req;

  usb_transfer_t *x = nullptr;
  if (usb_host_transfer_alloc(total, 0, &x) != ESP_OK) {
    ESP_LOGW(TAG, "[rdesc] alloc failed");
    return false;
  }

  usb_setup_packet_t *sp = (usb_setup_packet_t *) x->data_buffer;
  sp->bmRequestType = 0x81; // IN, Standard(Class?), Interface
  sp->bRequest      = 0x06; // GET_DESCRIPTOR
  sp->wValue        = (uint16_t)((0x22 << 8) | 0); // Report Descriptor
  sp->wIndex        = interface_number;
  sp->wLength       = req;

  x->num_bytes = total;
  x->callback = ctrl_noop_cb;
  x->context = nullptr;
  x->device_handle = dev_handle;
  x->bEndpointAddress = 0x00;
  x->flags = 0;

  if (usb_host_transfer_submit_control(client, x) != ESP_OK) {
    ESP_LOGW(TAG, "[rdesc] submit failed");
    usb_host_transfer_free(x);
    return false;
  }

  bool ok = wait_ctrl_done_(client, x, 1500);
  if (!ok) {
    ESP_LOGW(TAG, "[rdesc] status=0x%X", (unsigned) x->status);
    usb_host_transfer_free(x);
    return false;
  }

  int got = x->actual_num_bytes - USB_SETUP_PACKET_SIZE;
  if (got < 0) got = 0;
  out_len = (uint16_t) got;
  ESP_LOGI(TAG, "[rdesc] len=%d bytes", got);

  const uint8_t *rd = x->data_buffer + USB_SETUP_PACKET_SIZE;

  // Log chunked: 16 bytes por línea + yields cortos
  for (int i = 0; i < got; i += 16) {
    int n = (got - i) > 16 ? 16 : (got - i);
    log_hex_line_("[rdesc]", rd + i, n);
    vTaskDelay(pdMS_TO_TICKS(2));
  }

  usb_host_transfer_free(x);
  return true;
}

bool UpsHid::hid_get_report_input_ctrl_(
    usb_host_client_handle_t client,
    usb_device_handle_t dev_handle,
    uint8_t report_id,
    uint8_t *out_buf, int out_cap, int &out_len,
    uint8_t hid_if) {

  out_len = 0;
  if (!client || !dev_handle || !out_buf || out_cap <= 0) return false;

  // HID GET_REPORT(Input)
  const int want = out_cap;
  const int total = USB_SETUP_PACKET_SIZE + want;

  usb_transfer_t *x = nullptr;
  if (usb_host_transfer_alloc(total, 0, &x) != ESP_OK) {
    ESP_LOGW(TAG, "[poll] alloc failed");
    return false;
  }

  usb_setup_packet_t *sp = (usb_setup_packet_t *) x->data_buffer;
  sp->bmRequestType = 0xA1;      // IN | Class | Interface
  sp->bRequest      = 0x01;      // GET_REPORT
  sp->wValue        = (uint16_t)((0x01 << 8) | report_id); // 0x01 Input
  sp->wIndex        = hid_if;    // interfaz HID
  sp->wLength       = want;

  x->num_bytes = total;
  x->callback = ctrl_noop_cb;
  x->context = nullptr;
  x->device_handle = dev_handle;
  x->bEndpointAddress = 0x00;
  x->flags = 0;

  if (usb_host_transfer_submit_control(client, x) != ESP_OK) {
    ESP_LOGW(TAG, "[poll] submit failed");
    usb_host_transfer_free(x);
    return false;
  }

  bool ok = wait_ctrl_done_(client, x, 500);
  if (ok) {
    int got = x->actual_num_bytes - USB_SETUP_PACKET_SIZE;
    if (got < 0) got = 0;
    if (got > out_cap) got = out_cap;
    memcpy(out_buf, x->data_buffer + USB_SETUP_PACKET_SIZE, got);
    out_len = got;
  } else {
    ESP_LOGW(TAG, "[poll] ctrl status=0x%X", (unsigned) x->status);
  }

  usb_host_transfer_free(x);
  return ok;
}

// ------------------------------------------
// Component
// ------------------------------------------
void UpsHid::setup() {
  // 1) Instalar librería USB Host
  usb_host_config_t cfg = {.skip_phy_setup = false, .intr_flags = 0};
  esp_err_t err = usb_host_install(&cfg);
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "USB Host Library installed.");
  } else {
    ESP_LOGE(TAG, "usb_host_install() failed: 0x%X", (unsigned) err);
    return;
  }

  // 2) Tarea daemon
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

  // 4) Tarea que despacha eventos cliente
  xTaskCreatePinnedToCore(UpsHid::client_task_, "usbh_client",
                          4096, this, 5, nullptr, tskNO_AFFINITY);

  g_self = this;
}

void UpsHid::dump_config() {
  ESP_LOGCONFIG(TAG, "UPS HID component is configured.");
  ESP_LOGI(TAG, "UPS Host init step ready (no HID yet).");
}

// ------------------------------------------
// Tareas/callback
// ------------------------------------------
void UpsHid::host_daemon_task_(void *arg) {
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

  // Búfer de polling
  uint8_t buf[64];
  int     got = 0;

  while (true) {
    esp_err_t err = usb_host_client_handle_events(self->client_, pdMS_TO_TICKS(100));
    if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
      ESP_LOGW(TAG, "[usbh_client] handle_events err=0x%X", (unsigned) err);
    }

    // A) Descubrimiento tras NEW_DEV
    if (self->probe_pending_ && self->dev_handle_ != nullptr) {
      uint8_t if_num, ep; uint16_t mps; uint8_t itv; uint16_t cfg_total;
      if (read_config_descriptor_and_log_hid_(self->client_, self->dev_handle_,
                                              if_num, ep, mps, itv, cfg_total)) {
        self->hid_if_ = if_num;
        self->hid_ep_in_ = ep;
        self->hid_ep_mps_ = mps;
        self->hid_ep_interval_ = itv;
        ESP_LOGI(TAG, "[cfg] ready: IF=%u EP=0x%02X MPS=%u interval=%u",
                 (unsigned) self->hid_if_, (unsigned) self->hid_ep_in_,
                 (unsigned) self->hid_ep_mps_, (unsigned) self->hid_ep_interval_);

        // Volcar descriptor de reporte en trozos
        uint16_t rdlen = 0;
        (void) dump_report_descriptor_chunked_(self->client_, self->dev_handle_,
                                               self->hid_if_, rdlen);
      }
      self->probe_pending_ = false;
    }

    // B) Polling “manual” cada ~1000 ms (IDs: 0x01 y 0x64)
    static uint32_t last_ms = 0;
    uint32_t now_ms = (uint32_t) (esp_timer_get_time() / 1000ULL);
    if (self->dev_handle_ && self->hid_if_ != 0xFF && (now_ms - last_ms) >= 1000) {
      last_ms = now_ms;

      const uint8_t ids[2] = {0x01, 0x64};
      for (uint8_t rid : ids) {
        got = 0;
        if (hid_get_report_input_ctrl_(self->client_, self->dev_handle_,
                                       rid, buf, sizeof(buf), got, self->hid_if_)) {
          if (got > 0) {
            // log compacto (hasta 32 bytes)
            char line[3 * 32 + 1];
            int k = 0;
            int tolog = got > 32 ? 32 : got;
            for (int i = 0; i < tolog; i++) {
              k += snprintf(line + k, sizeof(line) - k, "%02X%s", buf[i], (i + 1 < tolog ? " " : ""));
              if (k >= (int)sizeof(line)) break;
            }
            ESP_LOGI(TAG, "[poll] GET_REPORT id=0x%02X len=%d data=%s%s",
                     rid, got, line, (got > tolog ? " ..." : ""));
            self->store_last_report_(rid, buf, got);
            // TODO: aquí añadiremos el parser real por campos
          }
        }
      }
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
        self->probe_pending_ = true;
      } else {
        ESP_LOGW(TAG, "[attach] NEW_DEV addr=%u but open failed: 0x%X",
                 (unsigned) msg->new_dev.address, (unsigned) e);
      }
      break;
    }
    case USB_HOST_CLIENT_EVENT_DEV_GONE: {
      self->dev_addr_ = 0;
      self->hid_if_ = 0xFF;
      self->hid_ep_in_ = 0;
      self->hid_ep_mps_ = 0;
      self->hid_ep_interval_ = 0;
      if (self->dev_handle_) {
        usb_host_device_close(self->client_, self->dev_handle_);
        self->dev_handle_ = nullptr;
      }
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

#include "ups_hid.h"

#include <cstring>  // memcpy, memset, snprintf

namespace esphome {
namespace ups_hid {

static const char *const TAG = "ups_hid";

// Estado global mínimo para callbacks del stack USB
static UpsHid *g_self = nullptr;

// -----------------------------------------------------
// Callbacks requeridos por IDF 5.4.x (no usamos interrupciones puras)
// -----------------------------------------------------
static void ctrl_transfer_cb_(usb_transfer_t *transfer) { (void) transfer; }

// =====================================================
/* Helpers de control transfer */
// =====================================================

bool UpsHid::get_config_header_(usb_host_client_handle_t client, usb_device_handle_t dev, uint8_t *cfg_hdr_out_9) {
  if (!client || !dev || !cfg_hdr_out_9) return false;

  const int hdr_len = 9;
  const int tot_len = USB_SETUP_PACKET_SIZE + hdr_len;

  usb_transfer_t *x = nullptr;
  if (usb_host_transfer_alloc(tot_len, 0, &x) != ESP_OK) return false;

  auto *setup = reinterpret_cast<usb_setup_packet_t *>(x->data_buffer);
  setup->bmRequestType = 0x80;                          // IN | Standard | Device
  setup->bRequest      = 0x06;                          // GET_DESCRIPTOR
  setup->wValue        = (uint16_t)((2 /*CONFIG*/ << 8) | 0);
  setup->wIndex        = 0;
  setup->wLength       = hdr_len;

  x->num_bytes        = tot_len;
  x->callback         = ctrl_transfer_cb_;
  x->context          = nullptr;
  x->device_handle    = dev;
  x->bEndpointAddress = 0x00;
  x->flags            = 0;

  esp_err_t e = usb_host_transfer_submit_control(client, x);
  if (e != ESP_OK) {
    usb_host_transfer_free(x);
    return false;
  }

  TickType_t dl = xTaskGetTickCount() + pdMS_TO_TICKS(1500);
  while (xTaskGetTickCount() < dl) {
    (void) usb_host_client_handle_events(client, pdMS_TO_TICKS(10));
    if (x->status == USB_TRANSFER_STATUS_COMPLETED ||
        x->status == USB_TRANSFER_STATUS_ERROR ||
        x->status == USB_TRANSFER_STATUS_STALL ||
        x->status == USB_TRANSFER_STATUS_NO_DEVICE ||
        x->status == USB_TRANSFER_STATUS_CANCELED) {
      break;
    }
  }

  bool ok = (x->status == USB_TRANSFER_STATUS_COMPLETED);
  if (ok) {
    const uint8_t *p = x->data_buffer + USB_SETUP_PACKET_SIZE;
    if (p[1] == 2 && p[0] >= 9) {  // CONFIG
      memcpy(cfg_hdr_out_9, p, 9);
    } else {
      ok = false;
    }
  }
  usb_host_transfer_free(x);
  return ok;
}

bool UpsHid::get_full_config_(usb_host_client_handle_t client, usb_device_handle_t dev, uint8_t *buf, int buf_len, int &out_len) {
  out_len = 0;
  if (!client || !dev || !buf || buf_len < 9) return false;

  uint8_t hdr[9];
  if (!get_config_header_(client, dev, hdr)) return false;

  uint16_t wTotalLength = (uint16_t)(hdr[2] | (hdr[3] << 8));
  if (wTotalLength > buf_len) wTotalLength = buf_len;

  const int tot_len = USB_SETUP_PACKET_SIZE + wTotalLength;

  usb_transfer_t *x = nullptr;
  if (usb_host_transfer_alloc(tot_len, 0, &x) != ESP_OK) return false;

  auto *setup = reinterpret_cast<usb_setup_packet_t *>(x->data_buffer);
  setup->bmRequestType = 0x80;
  setup->bRequest      = 0x06;  // GET_DESCRIPTOR
  setup->wValue        = (uint16_t)((2 /*CONFIG*/ << 8) | 0);
  setup->wIndex        = 0;
  setup->wLength       = wTotalLength;

  x->num_bytes        = tot_len;
  x->callback         = ctrl_transfer_cb_;
  x->context          = nullptr;
  x->device_handle    = dev;
  x->bEndpointAddress = 0x00;
  x->flags            = 0;

  esp_err_t e = usb_host_transfer_submit_control(client, x);
  if (e != ESP_OK) {
    usb_host_transfer_free(x);
    return false;
  }

  TickType_t dl = xTaskGetTickCount() + pdMS_TO_TICKS(2000);
  while (xTaskGetTickCount() < dl) {
    (void) usb_host_client_handle_events(client, pdMS_TO_TICKS(10));
    if (x->status == USB_TRANSFER_STATUS_COMPLETED ||
        x->status == USB_TRANSFER_STATUS_ERROR ||
        x->status == USB_TRANSFER_STATUS_STALL ||
        x->status == USB_TRANSFER_STATUS_NO_DEVICE ||
        x->status == USB_TRANSFER_STATUS_CANCELED) {
      break;
    }
  }

  bool ok = (x->status == USB_TRANSFER_STATUS_COMPLETED);
  if (ok) {
    const uint8_t *p = x->data_buffer + USB_SETUP_PACKET_SIZE;
    memcpy(buf, p, wTotalLength);
    out_len = wTotalLength;
  }
  usb_host_transfer_free(x);
  return ok;
}

bool UpsHid::read_config_descriptor_and_log_hid_(usb_host_client_handle_t client, usb_device_handle_t dev_handle,
                                                 uint8_t &if_num, uint8_t &ep_in,
                                                 uint16_t &mps, uint8_t &interval, uint16_t &rdesc_len) {
  if_num = 0xFF; ep_in = 0; mps = 0; interval = 0; rdesc_len = 0;
  if (!client || !dev_handle) return false;

  uint8_t cfg[512];
  int cfg_len = 0;
  if (!get_full_config_(client, dev_handle, cfg, sizeof(cfg), cfg_len)) {
    ESP_LOGW(TAG, "[cfg] get_full_config failed");
    return false;
  }

  const uint8_t *p = cfg;
  const uint8_t *end = cfg + cfg_len;
  int current_if = -1;

  while (p + 2 <= end && p[0] >= 2 && p + p[0] <= end) {
    uint8_t len = p[0], type = p[1];

    if (type == 4 /*INTERFACE*/ && len >= 9) {
      current_if = p[2];
      uint8_t bClass = p[5], bSub = p[6], bProto = p[7];
      if (bClass == 0x03 /*HID*/ && if_num == 0xFF) {
        if_num = (uint8_t) current_if;
        ESP_LOGI(TAG, "[cfg] HID IF=%d class=0x%02X sub=0x%02X proto=0x%02X",
                 (int) if_num, bClass, bSub, bProto);
      }
    } else if (type == 0x21 /*HID*/ && len >= 6) {
      if (if_num != 0xFF && current_if == (int) if_num) {
        if (len >= 9 && p[5] == 0x22) {  // Report descriptor type
          rdesc_len = (uint16_t)(p[6] | (p[7] << 8));
        }
      }
    } else if (type == 5 /*ENDPOINT*/ && len >= 7) {
      if (if_num != 0xFF && current_if == (int) if_num && ep_in == 0) {
        uint8_t bEndpointAddress = p[2];
        bool is_in = (bEndpointAddress & 0x80) != 0;
        bool is_intr = ((p[3] & 0x03) == 3);
        if (is_in && is_intr) {
          ep_in    = bEndpointAddress;
          mps      = (uint16_t)(p[4] | (p[5] << 8));
          interval = p[6];
          ESP_LOGI(TAG, "[cfg] HID endpoint IN=0x%02X MPS=%u interval=%u ms", ep_in, (unsigned) mps, (unsigned) interval);
        }
      }
    }

    p += len;
  }

  if (if_num != 0xFF && ep_in != 0) return true;
  ESP_LOGW(TAG, "[cfg] No se encontró interfaz HID o endpoint IN.");
  return false;
}

bool UpsHid::get_report_descriptor_(usb_host_client_handle_t client, usb_device_handle_t dev_handle,
                                    uint8_t if_num, uint8_t *buf, int buf_len, int &out_len) {
  out_len = 0;
  if (!client || !dev_handle || !buf || buf_len <= 0) return false;

  const int tot_len = USB_SETUP_PACKET_SIZE + buf_len;

  usb_transfer_t *x = nullptr;
  if (usb_host_transfer_alloc(tot_len, 0, &x) != ESP_OK) return false;

  auto *setup = reinterpret_cast<usb_setup_packet_t *>(x->data_buffer);
  setup->bmRequestType = 0x81;                   // IN | Standard | Interface
  setup->bRequest      = 0x06;                   // GET_DESCRIPTOR
  setup->wValue        = (uint16_t)((0x22 /*REPORT*/ << 8) | 0);
  setup->wIndex        = if_num;                 // interface
  setup->wLength       = buf_len;

  x->num_bytes        = tot_len;
  x->callback         = ctrl_transfer_cb_;
  x->context          = nullptr;
  x->device_handle    = dev_handle;
  x->bEndpointAddress = 0x00;
  x->flags            = 0;

  esp_err_t e = usb_host_transfer_submit_control(client, x);
  if (e != ESP_OK) {
    usb_host_transfer_free(x);
    return false;
  }

  TickType_t dl = xTaskGetTickCount() + pdMS_TO_TICKS(2000);
  while (xTaskGetTickCount() < dl) {
    (void) usb_host_client_handle_events(client, pdMS_TO_TICKS(10));
    if (x->status == USB_TRANSFER_STATUS_COMPLETED ||
        x->status == USB_TRANSFER_STATUS_ERROR ||
        x->status == USB_TRANSFER_STATUS_STALL ||
        x->status == USB_TRANSFER_STATUS_NO_DEVICE ||
        x->status == USB_TRANSFER_STATUS_CANCELED) {
      break;
    }
  }

  bool ok = (x->status == USB_TRANSFER_STATUS_COMPLETED);
  if (ok) {
    const uint8_t *p = x->data_buffer + USB_SETUP_PACKET_SIZE;
    memcpy(buf, p, buf_len);
    out_len = buf_len;  // el host suele truncar si pidió más de la cuenta
  }
  usb_host_transfer_free(x);
  return ok;
}

void UpsHid::dump_report_descriptor_(const uint8_t *buf, int len) {
  if (!buf || len <= 0) return;
  ESP_LOGI(TAG, "[rdesc] len=%d bytes", len);
  for (int i = 0; i < len; i += 16) {
    char line[3 * 16 + 1];
    int k = 0;
    int n = ((i + 16) <= len) ? 16 : (len - i);
    for (int j = 0; j < n; j++) {
      k += snprintf(line + k, sizeof(line) - k, "%02X%s", buf[i + j], (j + 1 < n ? " " : ""));
      if (k >= (int) sizeof(line)) break;
    }
    ESP_LOGI(TAG, "[rdesc] %s", line);
  }
}

bool UpsHid::hid_get_report_input_ctrl_(usb_host_client_handle_t client, usb_device_handle_t dev_handle,
                                        uint8_t if_num, uint8_t report_id,
                                        uint8_t *out_buf, int out_cap, int &out_len) {
  out_len = 0;
  if (!client || !dev_handle || !out_buf || out_cap <= 0) return false;

  const int tot_len = USB_SETUP_PACKET_SIZE + out_cap;

  usb_transfer_t *x = nullptr;
  if (usb_host_transfer_alloc(tot_len, 0, &x) != ESP_OK) return false;

  auto *setup = reinterpret_cast<usb_setup_packet_t *>(x->data_buffer);
  setup->bmRequestType = 0xA1;               // IN | Class | Interface
  setup->bRequest      = 0x01;               // GET_REPORT
  setup->wValue        = (uint16_t)((0x01 /*Input*/ << 8) | report_id);
  setup->wIndex        = if_num;             // *** YA NO USAMOS this ***
  setup->wLength       = out_cap;

  x->num_bytes        = tot_len;
  x->callback         = ctrl_transfer_cb_;
  x->context          = nullptr;
  x->device_handle    = dev_handle;
  x->bEndpointAddress = 0x00;
  x->flags            = 0;

  esp_err_t e = usb_host_transfer_submit_control(client, x);
  if (e != ESP_OK) {
    usb_host_transfer_free(x);
    return false;
  }

  TickType_t dl = xTaskGetTickCount() + pdMS_TO_TICKS(300);
  while (xTaskGetTickCount() < dl) {
    (void) usb_host_client_handle_events(client, pdMS_TO_TICKS(5));
    if (x->status == USB_TRANSFER_STATUS_COMPLETED ||
        x->status == USB_TRANSFER_STATUS_ERROR ||
        x->status == USB_TRANSFER_STATUS_STALL ||
        x->status == USB_TRANSFER_STATUS_NO_DEVICE ||
        x->status == USB_TRANSFER_STATUS_CANCELED) {
      break;
    }
  }

  bool ok = (x->status == USB_TRANSFER_STATUS_COMPLETED);
  if (ok) {
    const uint8_t *p = x->data_buffer + USB_SETUP_PACKET_SIZE;
    memcpy(out_buf, p, out_cap);
    out_len = out_cap;
  }
  usb_host_transfer_free(x);
  return ok;
}

// =====================================================
// Métodos de la clase
// =====================================================

void UpsHid::setup() {
  usb_host_config_t cfg = {.skip_phy_setup = false, .intr_flags = 0};
  esp_err_t err = usb_host_install(&cfg);
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "USB Host Library installed.");
  } else {
    ESP_LOGE(TAG, "usb_host_install() failed: 0x%X", (unsigned) err);
    return;
  }

  xTaskCreatePinnedToCore(UpsHid::host_daemon_task_, "usbh_daemon",
                          4096, nullptr, 5, nullptr, tskNO_AFFINITY);

  usb_host_client_config_t client_cfg = {
      .is_synchronous = false,
      .max_num_event_msg = 8,
      .async = {
          .client_event_callback = UpsHid::client_callback_,
          .callback_arg = this,
      },
  };
  err = usb_host_client_register(&client_cfg, &this->client_);
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "USB Host client registered.");
    xTaskCreatePinnedToCore(UpsHid::client_task_, "usbh_client",
                            4096, this, 5, nullptr, tskNO_AFFINITY);
  } else {
    ESP_LOGE(TAG, "usb_host_client_register() failed: 0x%X", (unsigned) err);
    return;
  }

  g_self = this;
}

void UpsHid::dump_config() {
  ESP_LOGCONFIG(TAG, "UPS HID component is configured.");
  ESP_LOGI(TAG, "UPS Host init step ready (no HID yet).");
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

  TickType_t last_tick = xTaskGetTickCount();

  while (true) {
    esp_err_t err = usb_host_client_handle_events(self->client_, pdMS_TO_TICKS(100));
    if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
      ESP_LOGW(TAG, "[usbh_client] handle_events err=0x%X", (unsigned) err);
    }

    // Descubrimiento tras NEW_DEV
    if (self->probe_pending_ && self->dev_handle_ != nullptr) {
      uint8_t if_num; uint8_t ep_in; uint16_t mps; uint8_t interval; uint16_t rdlen;
      if (read_config_descriptor_and_log_hid_(self->client_, self->dev_handle_, if_num, ep_in, mps, interval, rdlen)) {
        self->hid_if_              = if_num;
        self->hid_ep_in_           = ep_in;
        self->hid_ep_mps_          = mps;
        self->hid_ep_interval_     = interval;
        self->hid_report_desc_len_ = rdlen;

        ESP_LOGI(TAG, "[cfg] ready: IF=%u EP=0x%02X MPS=%u interval=%u",
                 (unsigned) self->hid_if_, (unsigned) self->hid_ep_in_,
                 (unsigned) self->hid_ep_mps_, (unsigned) self->hid_ep_interval_);

        // Dump report descriptor una vez
        if (!self->rdump_done_) {
          int want = (self->hid_report_desc_len_ > 0) ? self->hid_report_desc_len_ : 512;
          if (want > 1024) want = 1024;
          uint8_t *tmp = (uint8_t *) heap_caps_malloc(want, MALLOC_CAP_8BIT);
          if (tmp) {
            int got = 0;
            if (get_report_descriptor_(self->client_, self->dev_handle_, self->hid_if_, tmp, want, got) && got > 0) {
              dump_report_descriptor_(tmp, got);
            }
            heap_caps_free(tmp);
          }
          self->rdump_done_ = true;
        }
      }
      self->probe_pending_ = false;
    }

    // Sondeo ~1 Hz
    TickType_t now_tick = xTaskGetTickCount();
    if (self->dev_handle_ != nullptr && self->hid_if_ != 0xFF &&
        (now_tick - last_tick) >= pdMS_TO_TICKS(1000)) {
      last_tick = now_tick;

      const uint8_t ids[] = {0x01, 0x64, 0x66};
      uint8_t buf[64];
      for (uint8_t id : ids) {
        int got = 0;
        memset(buf, 0, sizeof(buf));
        if (UpsHid::hid_get_report_input_ctrl_(self->client_, self->dev_handle_, self->hid_if_, id,
                                               buf, sizeof(buf), got) && got > 0) {
          char line[3 * 64 + 1];
          int k = 0;
          for (int i = 0; i < got; i++) {
            k += snprintf(line + k, sizeof(line) - k, "%02X%s", buf[i], (i + 1 < got ? " " : ""));
            if (k >= (int) sizeof(line)) break;
          }
          ESP_LOGI(TAG, "[poll] GET_REPORT id=0x%02X len=%d data=%s", (unsigned) id, got, line);
          // TODO: parseo y publicación de sensores
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
        self->hid_if_   = 0xFF;
        self->hid_ep_in_ = 0;
        self->hid_ep_mps_ = 0;
        self->hid_ep_interval_ = 0;
        self->hid_report_desc_len_ = 0;
        self->rdump_done_ = false;
        self->probe_pending_ = true;
        ESP_LOGI(TAG, "[attach] NEW_DEV addr=%u (opened)", (unsigned) self->dev_addr_);
      } else {
        ESP_LOGW(TAG, "[attach] NEW_DEV addr=%u but open failed: 0x%X",
                 (unsigned) msg->new_dev.address, (unsigned) e);
      }
      break;
    }
    case USB_HOST_CLIENT_EVENT_DEV_GONE: {
      if (self->dev_handle_ != nullptr) {
        usb_host_device_close(self->client_, self->dev_handle_);
        self->dev_handle_ = nullptr;
      }
      self->dev_addr_ = 0;
      self->hid_if_   = 0xFF;
      self->hid_ep_in_ = 0;
      self->hid_ep_mps_ = 0;
      self->hid_ep_interval_ = 0;
      self->hid_report_desc_len_ = 0;
      self->rdump_done_ = false;
      self->probe_pending_ = false;
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

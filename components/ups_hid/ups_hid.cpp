#include "ups_hid.h"
#include "usb/usb_types_ch9.h"  // usb_setup_packet_t, USB_SETUP_PACKET_SIZE

namespace esphome {
namespace ups_hid {

static const char *const TAG = "ups_hid";

// -----------------------------------------------------
// Estado simple + flags para trabajo fuera del callback
// -----------------------------------------------------
static UpsHid *g_self = nullptr;
static volatile bool g_probe_pending = false;   // lanzar descubrimiento fuera del callback

// callback no-op para transfers de control
static void ctrl_transfer_cb_(usb_transfer_t *transfer) { (void)transfer; }

// -----------------------------------------------------
// Helper A: lee Configuration Descriptor y obtiene IF HID y EP IN
// -----------------------------------------------------
static bool read_config_descriptor_and_log_hid_(usb_host_client_handle_t client,
                                                usb_device_handle_t dev_handle,
                                                uint8_t &if_num, uint8_t &ep_in,
                                                uint16_t &mps, uint8_t &interval) {
  if_num = 0; ep_in = 0; mps = 0; interval = 0;
  if (!client || !dev_handle) return false;

  // A) Header (9 bytes) de la config activa
  const int hdr_len = 9;
  const int hdr_tot = USB_SETUP_PACKET_SIZE + hdr_len;
  usb_transfer_t *xhdr = nullptr;
  if (usb_host_transfer_alloc(hdr_tot, 0, &xhdr) != ESP_OK) {
    ESP_LOGW(TAG, "[cfg] transfer_alloc header failed");
    return false;
  }
  usb_setup_packet_t *sh = (usb_setup_packet_t *) xhdr->data_buffer;
  sh->bmRequestType = 0x80;           // IN | standard | device
  sh->bRequest      = 0x06;           // GET_DESCRIPTOR
  sh->wValue        = (uint16_t)((2 << 8) | 0);  // CONFIGURATION(2) | index 0
  sh->wIndex        = 0;
  sh->wLength       = hdr_len;

  xhdr->num_bytes        = hdr_tot;
  xhdr->callback         = ctrl_transfer_cb_;
  xhdr->context          = nullptr;
  xhdr->device_handle    = dev_handle;
  xhdr->bEndpointAddress = 0x00;  // EP0
  xhdr->flags            = 0;

  if (usb_host_transfer_submit_control(client, xhdr) != ESP_OK) {
    ESP_LOGW(TAG, "[cfg] submit header failed");
    usb_host_transfer_free(xhdr);
    return false;
  }

  {
    TickType_t dl = xTaskGetTickCount() + pdMS_TO_TICKS(1500);
    while (xTaskGetTickCount() < dl) {
      (void) usb_host_client_handle_events(client, pdMS_TO_TICKS(10));
      if (xhdr->status == USB_TRANSFER_STATUS_COMPLETED ||
          xhdr->status == USB_TRANSFER_STATUS_ERROR ||
          xhdr->status == USB_TRANSFER_STATUS_STALL ||
          xhdr->status == USB_TRANSFER_STATUS_NO_DEVICE ||
          xhdr->status == USB_TRANSFER_STATUS_CANCELED) {
        break;
      }
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

  // B) Descriptor de configuración completo
  int payload = wTotalLength;
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

  {
    TickType_t dl = xTaskGetTickCount() + pdMS_TO_TICKS(2000);
    while (xTaskGetTickCount() < dl) {
      (void) usb_host_client_handle_events(client, pdMS_TO_TICKS(10));
      if (xfull->status == USB_TRANSFER_STATUS_COMPLETED ||
          xfull->status == USB_TRANSFER_STATUS_ERROR ||
          xfull->status == USB_TRANSFER_STATUS_STALL ||
          xfull->status == USB_TRANSFER_STATUS_NO_DEVICE ||
          xfull->status == USB_TRANSFER_STATUS_CANCELED) {
        break;
      }
    }
  }

  if (xfull->status != USB_TRANSFER_STATUS_COMPLETED) {
    ESP_LOGW(TAG, "[cfg] full status=0x%X", (unsigned) xfull->status);
    usb_host_transfer_free(xfull);
    return false;
  }

  // C) Parseo: localizar INTERFACE HID y su ENDPOINT IN interrupt
  const uint8_t *p   = xfull->data_buffer + USB_SETUP_PACKET_SIZE;
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
      bool is_in   = (bEndpointAddress & 0x80) != 0;
      bool is_intr = ((p[3] & 0x03) == 3);
      if (is_in && is_intr) {
        ep_in    = bEndpointAddress;
        mps      = (uint16_t)(p[4] | (p[5] << 8));
        interval = p[6];
      }
    }
    p += len;
  }

  usb_host_transfer_free(xfull);

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

// -----------------------------------------------------
// Helper B: HID GET_REPORT (Input) por EP0 (control)
// -----------------------------------------------------
static bool hid_get_report_input_ctrl_(usb_host_client_handle_t client,
                                       usb_device_handle_t dev_handle,
                                       uint8_t if_num,
                                       uint8_t report_id,
                                       uint8_t *out_buf, int out_len, int &got_len) {
  got_len = 0;
  if (!client || !dev_handle || !out_buf || out_len <= 0) return false;

  const int tot = USB_SETUP_PACKET_SIZE + out_len;
  usb_transfer_t *x = nullptr;
  if (usb_host_transfer_alloc(tot, 0, &x) != ESP_OK) {
    ESP_LOGW(TAG, "[poll] transfer_alloc failed");
    return false;
  }

  // HID GET_REPORT: bmRequestType=1010 0001b = 0xA1 (IN | Class | Interface)
  // bRequest = 0x01, wValue = (REPORT_TYPE_INPUT=1 << 8) | report_id
  usb_setup_packet_t *sp = (usb_setup_packet_t *) x->data_buffer;
  sp->bmRequestType = 0xA1;
  sp->bRequest      = 0x01;  // GET_REPORT
  sp->wValue        = (uint16_t)((1 << 8) | (report_id & 0xFF)); // Input, report_id
  sp->wIndex        = if_num;    // interfaz HID
  sp->wLength       = out_len;   // bytes esperados

  x->num_bytes        = tot;
  x->callback         = ctrl_transfer_cb_;
  x->context          = nullptr;
  x->device_handle    = dev_handle;
  x->bEndpointAddress = 0x00;    // EP0
  x->flags            = 0;

  if (usb_host_transfer_submit_control(client, x) != ESP_OK) {
    ESP_LOGW(TAG, "[poll] submit failed");
    usb_host_transfer_free(x);
    return false;
  }

  // Espera corta
  {
    TickType_t dl = xTaskGetTickCount() + pdMS_TO_TICKS(500);
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
  }

  bool ok = false;
  if (x->status == USB_TRANSFER_STATUS_COMPLETED) {
    const uint8_t *d = x->data_buffer + USB_SETUP_PACKET_SIZE;
    got_len = x->actual_num_bytes - USB_SETUP_PACKET_SIZE;
    if (got_len < 0) got_len = 0;
    if (got_len > out_len) got_len = out_len;
    memcpy(out_buf, d, got_len);
    ok = true;
  } else {
    ESP_LOGV(TAG, "[poll] status=0x%X", (unsigned) x->status);
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

void UpsHid::update() {
  // Poll ligero por control cada ~1s cuando hay dispositivo
  static uint32_t last_ms = 0;
  uint32_t now = millis();
  if (this->dev_handle_ && this->hid_if_ >= 0 && (now - last_ms) > 1000) {
    last_ms = now;
    uint8_t buf[32];
    int got = 0;
    // report_id 0 es lo más común en HID UPS genéricos
    if (hid_get_report_input_ctrl_(this->client_, this->dev_handle_,
                                   (uint8_t)this->hid_if_, 0, buf, sizeof(buf), got) && got > 0) {
      // Log HEX corto
      int max_log = got < 32 ? got : 32;
      char hex[3 * 32 + 1];
      int k = 0;
      for (int i = 0; i < max_log; i++) {
        k += snprintf(hex + k, sizeof(hex) - k, "%02X%s", buf[i], (i + 1 < max_log ? " " : ""));
        if (k >= (int)sizeof(hex)) break;
      }
      ESP_LOGI(TAG, "[poll] GET_REPORT len=%d data=%s%s",
               got, hex, (got > max_log ? " ..." : ""));
    }
  }

  if (!this->hello_logged_) {
    ESP_LOGI(TAG, "UPS HID component started (hello from ESPHome external component).");
    this->hello_logged_ = true;
  }
}

// =====================================================
// Funciones estáticas (tareas/callback)
// =====================================================

void UpsHid::host_daemon_task_(void *arg) {
  uint32_t flags = 0;
  while (true) {
    esp_err_t err = usb_host_lib_handle_events(pdMS_TO_TICKS(1000), &flags);
    if (err == ESP_OK) {
      if (flags) {
        ESP_LOGI(TAG, "[usbh_daemon] USB Host event flags: 0x%08X", (unsigned) flags);
      }
    } else if (err != ESP_ERR_TIMEOUT) {
      ESP_LOGW(TAG, "[usbh_daemon] handle_events err=0x%X", (unsigned) err);
    }
    flags = 0;
  }
}

void UpsHid::client_task_(void *arg) {
  auto *self = static_cast<UpsHid *>(arg);
  if (!self || !self->client_) {
    ESP_LOGE(TAG, "[usbh_client] No client handle.");
    vTaskDelete(nullptr);
    return;
  }
  while (true) {
    esp_err_t err = usb_host_client_handle_events(self->client_, pdMS_TO_TICKS(100));
    if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
      ESP_LOGW(TAG, "[usbh_client] handle_events err=0x%X", (unsigned) err);
    }

    // Descubrimiento HID (fuera del callback)
    if (g_probe_pending && self->dev_handle_ != nullptr) {
      uint8_t ifn, ep; uint16_t mps; uint8_t itv;
      if (read_config_descriptor_and_log_hid_(self->client_, self->dev_handle_, ifn, ep, mps, itv)) {
        self->hid_if_          = (int) ifn;   // guardamos nº de interfaz HID
        self->hid_ep_in_       = ep;
        self->hid_ep_mps_      = mps;
        self->hid_ep_interval_ = itv;
        ESP_LOGI(TAG, "[cfg] ready: IF=%u EP=0x%02X MPS=%u interval=%u",
                 (unsigned) ifn, (unsigned) ep, (unsigned) mps, (unsigned) itv);
      }
      g_probe_pending = false;
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
        self->hid_if_   = -1;          // aún no detectado
        self->hid_ep_in_ = 0;          // aún no detectado
        ESP_LOGI(TAG, "[attach] NEW_DEV addr=%u (opened)", (unsigned) self->dev_addr_);
        g_probe_pending = true;        // lanzar descubrimiento
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
      self->dev_addr_   = 0;
      self->hid_if_     = -1;
      self->hid_ep_in_  = 0;
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

#include "ups_hid.h"

namespace esphome {
namespace ups_hid {

static const char *const TAG = "ups_hid";

// -----------------------------------------------------
// Estado simple + callbacks requeridos por IDF 5.4.x
// -----------------------------------------------------
static UpsHid *g_self = nullptr;
static volatile bool g_probe_pending = false;   // lanzar descubrimiento fuera del callback
static volatile bool g_start_listen = false;    // arrancar lectura EP IN fuera del callback

// callback no-op para transfers de control
static void ctrl_transfer_cb_(usb_transfer_t *transfer) { (void)transfer; }

// callback de la transferencia IN (endpoint interrupt)
void in_transfer_cb_(usb_transfer_t *xfer) {
  // Ojo: estamos en contexto de ISR/task del stack USB. Hacemos logs breves.
  if (!g_self) return;
  switch (xfer->status) {
    case USB_TRANSFER_STATUS_COMPLETED: {
      // El payload empieza en data_buffer (no hay SETUP).
      const uint8_t *d = xfer->data_buffer;
      int n = xfer->actual_num_bytes;
      // Log corto en hex (hasta 32 bytes para no saturar)
      int max_log = n < 32 ? n : 32;
      char buf[3 * 32 + 1];
      int k = 0;
      for (int i = 0; i < max_log; i++) {
        k += snprintf(buf + k, sizeof(buf) - k, "%02X%s", d[i], (i + 1 < max_log ? " " : ""));
        if (k >= (int)sizeof(buf)) break;
      }
      ESP_LOGI(TAG, "[in] report len=%d data=%s%s", n, buf, (n > max_log ? " ..." : ""));
      // Volver a enviar para la siguiente lectura
      (void) usb_host_transfer_submit(g_self->in_xfer_);
      break;
    }
    case USB_TRANSFER_STATUS_NO_DEVICE:
      ESP_LOGI(TAG, "[in] NO_DEVICE (desconectado)");
      g_self->listening_ = false;
      break;
    case USB_TRANSFER_STATUS_STALL:
    case USB_TRANSFER_STATUS_ERROR:
    case USB_TRANSFER_STATUS_CANCELED:
    default:
      ESP_LOGW(TAG, "[in] status=0x%X", (unsigned) xfer->status);
      // Reintento suave si seguimos conectados
      if (g_self->dev_handle_ && g_self->listening_) {
        (void) usb_host_transfer_submit(g_self->in_xfer_);
      }
      break;
  }
}

// -----------------------------------------------------
// Helper: lee Configuration Descriptor, rellena ep_in/mps/interval y loguea
// -----------------------------------------------------
static bool read_config_descriptor_and_log_hid_(usb_host_client_handle_t client,
                                                usb_device_handle_t dev_handle,
                                                uint8_t &ep_in, uint16_t &mps, uint8_t &interval) {
  ep_in = 0; mps = 0; interval = 0;
  if (!client || !dev_handle) return false;

  // A) Header (9 bytes)
  const int hdr_len = 9;
  const int hdr_tot = USB_SETUP_PACKET_SIZE + hdr_len;
  usb_transfer_t *xhdr = nullptr;
  if (usb_host_transfer_alloc(hdr_tot, 0, &xhdr) != ESP_OK) {
    ESP_LOGW(TAG, "[cfg] transfer_alloc header failed");
    return false;
  }
  usb_setup_packet_t *sh = (usb_setup_packet_t *) xhdr->data_buffer;
  sh->bmRequestType = 0x80; sh->bRequest = 0x06; sh->wValue = (uint16_t)((2 << 8) | 0);
  sh->wIndex = 0; sh->wLength = hdr_len;

  xhdr->num_bytes = hdr_tot; xhdr->callback = ctrl_transfer_cb_;
  xhdr->context = nullptr; xhdr->device_handle = dev_handle;
  xhdr->bEndpointAddress = 0x00; xhdr->flags = 0;

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

  // B) Descriptor completo
  int payload = wTotalLength;
  int total   = USB_SETUP_PACKET_SIZE + payload;
  usb_transfer_t *xfull = nullptr;
  if (usb_host_transfer_alloc(total, 0, &xfull) != ESP_OK) {
    ESP_LOGW(TAG, "[cfg] transfer_alloc full failed");
    return false;
  }
  usb_setup_packet_t *sf = (usb_setup_packet_t *) xfull->data_buffer;
  sf->bmRequestType = 0x80; sf->bRequest = 0x06; sf->wValue = (uint16_t)((2 << 8) | 0);
  sf->wIndex = 0; sf->wLength = payload;

  xfull->num_bytes = total; xfull->callback = ctrl_transfer_cb_;
  xfull->context = nullptr; xfull->device_handle = dev_handle;
  xfull->bEndpointAddress = 0x00; xfull->flags = 0;

  if (usb_host_transfer_submit_control(client, xfull) != ESP_OK) {
    ESP_LOGW(TAG, "[cfg] submit full failed");
    usb_host_transfer_free(xfull);
    return false;
  }
  {
    TickType_t dl = xTaskGetTickCount() + pdMS_TO_TICKS(1500);
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

  // C) Parseo: INTERFACE HID + ENDPOINT IN interrupt
  const uint8_t *p = xfull->data_buffer + USB_SETUP_PACKET_SIZE;
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

  if (hid_if >= 0 && ep_in != 0) {
    ESP_LOGI(TAG, "[cfg] HID endpoint IN=0x%02X MPS=%u interval=%u ms",
             ep_in, (unsigned) mps, (unsigned) interval);
    usb_host_transfer_free(xfull);
    return true;
  } else {
    ESP_LOGW(TAG, "[cfg] No se encontró interfaz HID o endpoint IN.");
    usb_host_transfer_free(xfull);
    return false;
  }
}

// =====================================================
// Métodos de la clase
// =====================================================

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
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "USB Host client registered.");
    // 4) Tarea que despacha eventos
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
  while (true) {
    esp_err_t err = usb_host_client_handle_events(self->client_, pdMS_TO_TICKS(100));
    if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
      ESP_LOGW(TAG, "[usbh_client] handle_events err=0x%X", (unsigned) err);
    }

    // 1) Descubrimiento HID (fuera del callback)
    if (g_probe_pending && self->dev_handle_ != nullptr) {
      uint8_t ep; uint16_t mps; uint8_t itv;
      if (read_config_descriptor_and_log_hid_(self->client_, self->dev_handle_, ep, mps, itv)) {
        self->hid_ep_in_ = ep;
        self->hid_ep_mps_ = mps;
        self->hid_ep_interval_ = itv;
        g_start_listen = true;  // listo para empezar lectura de reports
      }
      g_probe_pending = false;
    }

    // 2) Arrancar lectura continua del EP IN (una vez descubierto)
    if (g_start_listen && !self->listening_ && self->dev_handle_ != nullptr && self->hid_ep_in_ != 0) {
      // Reservar/armar transfer
      int size = self->hid_ep_mps_;
      if (size <= 0 || size > 64) size = 64;  // por seguridad
      if (self->in_xfer_ == nullptr) {
        if (usb_host_transfer_alloc(size, 0, &self->in_xfer_) != ESP_OK) {
          ESP_LOGW(TAG, "[in] transfer_alloc failed");
        }
      }
      if (self->in_xfer_) {
        self->in_xfer_->num_bytes = size;
        self->in_xfer_->callback = in_transfer_cb_;
        self->in_xfer_->context = nullptr;
        self->in_xfer_->device_handle = self->dev_handle_;
        self->in_xfer_->bEndpointAddress = self->hid_ep_in_;
        self->in_xfer_->flags = 0;
        esp_err_t se = usb_host_transfer_submit(self->in_xfer_);
        if (se == ESP_OK) {
          self->listening_ = true;
          ESP_LOGI(TAG, "[in] listening on EP=0x%02X (MPS=%u, interval~%u ms)", self->hid_ep_in_, (unsigned) self->hid_ep_mps_, (unsigned) self->hid_ep_interval_);
        } else {
          ESP_LOGW(TAG, "[in] submit failed: 0x%X", (unsigned) se);
        }
      }
      g_start_listen = false;
    }
  }
}

void UpsHid::client_callback_(const usb_host_client_event_msg_t *msg, void *arg) {
  auto *self = static_cast<UpsHid *>(arg);
  if (!self) return;

  switch (msg->event) {
    case USB_HOST_CLIENT_EVENT_NEW_DEV: {
      // Abrir el dispositivo y guardar addr/handle
      esp_err_t e = usb_host_device_open(self->client_, msg->new_dev.address, &self->dev_handle_);
      if (e == ESP_OK) {
        self->dev_addr_ = msg->new_dev.address;
        ESP_LOGI(TAG, "[attach] NEW_DEV addr=%u (opened)", (unsigned) self->dev_addr_);
        // lanzamos descubrimiento fuera del callback
        g_probe_pending = true;
      } else {
        ESP_LOGW(TAG, "[attach] NEW_DEV addr=%u but open failed: 0x%X",
                 (unsigned) msg->new_dev.address, (unsigned) e);
      }
      break;
    }
    case USB_HOST_CLIENT_EVENT_DEV_GONE: {
      // Parar lectura si estaba activa
      self->listening_ = false;
      if (self->in_xfer_) {
        // no liberamos aquí para evitar free en ISR; se puede reciclar al re-attach
      }
      if (self->dev_handle_ != nullptr) {
        usb_host_device_close(self->client_, self->dev_handle_);
        self->dev_handle_ = nullptr;
      }
      self->dev_addr_ = 0;
      self->hid_ep_in_ = 0;
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

#include "ups_hid.h"

namespace esphome {
namespace ups_hid {

static const char *const TAG = "ups_hid";

// -----------------------------------------------------
// Estado simple + callback requerido por IDF 5.4.x
// -----------------------------------------------------
static UpsHid *g_self = nullptr;
static volatile bool g_probe_pending = false;  // marcar sonda HID fuera del callback

static void ctrl_transfer_cb_(usb_transfer_t *transfer) {
  (void) transfer;  // no-op
}

// -----------------------------------------------------
// Helper: lee el Configuration Descriptor y loguea HID+EP IN
// (se ejecuta desde client_task_, nunca desde el callback)
// -----------------------------------------------------
static bool read_config_descriptor_and_log_hid_(usb_host_client_handle_t client,
                                                usb_device_handle_t dev_handle) {
  if (!client || !dev_handle) return false;

  // A) Leer header (9 bytes) para sacar wTotalLength
  const int hdr_len = 9;
  const int hdr_tot = USB_SETUP_PACKET_SIZE + hdr_len;

  usb_transfer_t *xhdr = nullptr;
  if (usb_host_transfer_alloc(hdr_tot, 0, &xhdr) != ESP_OK) {
    ESP_LOGW(TAG, "[cfg] transfer_alloc header failed");
    return false;
  }

  usb_setup_packet_t *sh = (usb_setup_packet_t *) xhdr->data_buffer;
  sh->bmRequestType = 0x80;                   // IN | Standard | Device
  sh->bRequest      = 0x06;                   // GET_DESCRIPTOR
  sh->wValue        = (uint16_t)((2 << 8) | 0); // CONFIGURATION(2) << 8 | 0
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
    TickType_t dl = xTaskGetTickCount() + pdMS_TO_TICKS(1000);
    while (xTaskGetTickCount() < dl &&
           xhdr->status != USB_TRANSFER_STATUS_COMPLETED &&
           xhdr->status != USB_TRANSFER_STATUS_ERROR &&
           xhdr->status != USB_TRANSFER_STATUS_STALL &&
           xhdr->status != USB_TRANSFER_STATUS_NO_DEVICE &&
           xhdr->status != USB_TRANSFER_STATUS_CANCELED) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }

  if (xhdr->status != USB_TRANSFER_STATUS_COMPLETED) {
    ESP_LOGW(TAG, "[cfg] header status=0x%X", (unsigned) xhdr->status);
    usb_host_transfer_free(xhdr);
    return false;
  }

  const uint8_t *cfg_hdr = xhdr->data_buffer + USB_SETUP_PACKET_SIZE;
  if (cfg_hdr[1] != 2 || cfg_hdr[0] < 9) {
    ESP_LOGW(TAG, "[cfg] invalid config header (bType=%u bLen=%u)", cfg_hdr[1], cfg_hdr[0]);
    usb_host_transfer_free(xhdr);
    return false;
  }
  uint16_t wTotalLength = (uint16_t)(cfg_hdr[2] | (cfg_hdr[3] << 8));
  usb_host_transfer_free(xhdr);

  // B) Leer descriptor completo
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
    TickType_t dl = xTaskGetTickCount() + pdMS_TO_TICKS(1000);
    while (xTaskGetTickCount() < dl &&
           xfull->status != USB_TRANSFER_STATUS_COMPLETED &&
           xfull->status != USB_TRANSFER_STATUS_ERROR &&
           xfull->status != USB_TRANSFER_STATUS_STALL &&
           xfull->status != USB_TRANSFER_STATUS_NO_DEVICE &&
           xfull->status != USB_TRANSFER_STATUS_CANCELED) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }

  if (xfull->status != USB_TRANSFER_STATUS_COMPLETED) {
    ESP_LOGW(TAG, "[cfg] full status=0x%X", (unsigned) xfull->status);
    usb_host_transfer_free(xfull);
    return false;
  }

  // C) Parseo: INTERFACE (class=HID=0x03) + ENDPOINT IN interrupt
  const uint8_t *p = xfull->data_buffer + USB_SETUP_PACKET_SIZE;
  const uint8_t *end = p + payload;

  int hid_if = -1;
  uint8_t ep_in = 0;
  uint16_t ep_mps = 0;
  uint8_t ep_interval = 0;

  while (p + 2 <= end && p[0] >= 2 && p + p[0] <= end) {
    uint8_t len = p[0];
    uint8_t type = p[1];

    if (type == 4 && len >= 9) {  // INTERFACE
      uint8_t bInterfaceNumber = p[2];
      uint8_t bClass  = p[5];
      uint8_t bSub    = p[6];
      uint8_t bProto  = p[7];
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
        ep_in = bEndpointAddress;
        ep_mps = (uint16_t)(p[4] | (p[5] << 8));
        ep_interval = p[6];
      }
    }
    p += len;
  }

  if (hid_if >= 0 && ep_in != 0) {
    ESP_LOGI(TAG, "[cfg] HID endpoint IN=0x%02X MPS=%u interval=%u ms",
             ep_in, (unsigned) ep_mps, (unsigned) ep_interval);
  } else {
    ESP_LOGW(TAG, "[cfg] No se encontró interfaz HID o endpoint IN.");
  }

  usb_host_transfer_free(xfull);
  return (hid_if >= 0);
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

  // Guardar puntero global (una sola instancia)
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

    // Ejecutar la sonda HID FUERA del callback
    if (g_probe_pending) {
      if (self->dev_handle_ != nullptr) {
        (void) read_config_descriptor_and_log_hid_(self->client_, self->dev_handle_);
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
      // Abrir el dispositivo y guardar addr/handle
      esp_err_t e = usb_host_device_open(self->client_, msg->new_dev.address, &self->dev_handle_);
      if (e == ESP_OK) {
        self->dev_addr_ = msg->new_dev.address;
        ESP_LOGI(TAG, "[attach] NEW_DEV addr=%u (opened)", (unsigned) self->dev_addr_);
        // marcar sonda (se ejecutará en client_task_)
        g_probe_pending = true;
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

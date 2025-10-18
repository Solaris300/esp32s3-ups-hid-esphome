#include "ups_hid.h"
#include "usb/usb_types_ch9.h"   // usb_setup_packet_t, USB_SETUP_PACKET_SIZE

namespace esphome {
namespace ups_hid {

static const char *const TAG = "ups_hid";

// --- PLAN B: GET_DESCRIPTOR(Device) por control transfer (solo en este .cpp) ---
static bool read_device_descriptor_ctrl_(usb_host_client_handle_t client, usb_device_handle_t dev_handle) {
  if (!client || !dev_handle) return false;

  const int desc_len  = 18;
  const int total_len = USB_SETUP_PACKET_SIZE + desc_len;

  usb_transfer_t *xfer = nullptr;
  if (usb_host_transfer_alloc(total_len, 0, &xfer) != ESP_OK) {
    ESP_LOGE(TAG, "[dev] usb_host_transfer_alloc failed");
    return false;
  }

  // Setup packet: GET_DESCRIPTOR(Device, index 0)
  usb_setup_packet_t *setup = (usb_setup_packet_t *) xfer->data_buffer;
  setup->bmRequestType = 0x80;                      // IN | Standard | Device
  setup->bRequest      = 0x06;                      // GET_DESCRIPTOR
  setup->wValue        = (uint16_t)((1 << 8) | 0);  // DEVICE(1)<<8 | 0
  setup->wIndex        = 0;
  setup->wLength       = desc_len;

  xfer->num_bytes         = total_len;
  xfer->callback          = nullptr;
  xfer->context           = nullptr;
  xfer->device_handle     = dev_handle;   // <— IMPORTANTE
  xfer->bEndpointAddress  = 0x00;         // EP0 (control)
  xfer->flags             = 0;

  // Enviar control transfer: pasa el CLIENT handle (IDF 5.4.x)
  esp_err_t e = usb_host_transfer_submit_control(client, xfer);
  if (e != ESP_OK) {
    ESP_LOGE(TAG, "[dev] transfer_submit_control failed: 0x%X", (unsigned) e);
    usb_host_transfer_free(xfer);
    return false;
  }

  // Espera acotada a que el host complete
  const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(1000);
  while (xTaskGetTickCount() < deadline) {
    if (xfer->status == USB_TRANSFER_STATUS_COMPLETED ||
        xfer->status == USB_TRANSFER_STATUS_ERROR ||
        xfer->status == USB_TRANSFER_STATUS_STALL ||
        xfer->status == USB_TRANSFER_STATUS_NO_DEVICE ||
        xfer->status == USB_TRANSFER_STATUS_CANCELED) {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  if (xfer->status != USB_TRANSFER_STATUS_COMPLETED) {
    ESP_LOGW(TAG, "[dev] control transfer status=0x%X", (unsigned) xfer->status);
    usb_host_transfer_free(xfer);
    return false;
  }

  // Descriptor tras el setup (8 bytes)
  const uint8_t *d = xfer->data_buffer + USB_SETUP_PACKET_SIZE;
  if (d[0] < 18 || d[1] != 1) {
    ESP_LOGW(TAG, "[dev] invalid device descriptor header: bLen=%u bType=%u", d[0], d[1]);
    usb_host_transfer_free(xfer);
    return false;
  }

  uint16_t vid = (uint16_t)(d[8]  | (d[9]  << 8));
  uint16_t pid = (uint16_t)(d[10] | (d[11] << 8));
  uint8_t  cls = d[4], subcls = d[5], proto = d[6];
  ESP_LOGI(TAG, "[dev] VID=0x%04X PID=0x%04X class=0x%02X sub=0x%02X proto=0x%02X",
           vid, pid, cls, subcls, proto);

  usb_host_transfer_free(xfer);
  return true;
}


// ---------- Métodos de la clase ----------

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
    xTaskCreatePinnedToCore(UpsHid::client_task_, "usbh_client",
                            4096, this, 5, nullptr, tskNO_AFFINITY);
  } else {
    ESP_LOGE(TAG, "usb_host_client_register() failed: 0x%X", (unsigned) err);
  }
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

// ---------- Funciones estáticas (tareas/callback) ----------

void UpsHid::host_daemon_task_(void *arg) {
  uint32_t flags = 0;
  while (true) {
    esp_err_t err = usb_host_lib_handle_events(pdMS_TO_TICKS(1000), &flags);
    if (err == ESP_OK && flags) {
      ESP_LOGI(TAG, "[usbh_daemon] USB Host event flags: 0x%08X", (unsigned) flags);
      flags = 0;
    } else if (err == ESP_ERR_TIMEOUT) {
      // sin eventos
    } else if (err != ESP_OK) {
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
    // El callback se ejecuta cuando haya eventos
    esp_err_t err = usb_host_client_handle_events(self->client_, pdMS_TO_TICKS(1000));
    if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
      ESP_LOGW(TAG, "[usbh_client] handle_events err=0x%X", (unsigned) err);
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

        // --- A) Ruta “cómoda” si existe en tu IDF
#if ESP_IDF_VERSION_MAJOR >= 5
  #ifdef usb_host_device_info
        {
          usb_host_device_info_t dinfo{};
          esp_err_t ie = usb_host_device_info(self->dev_handle_, &dinfo);
          if (ie == ESP_OK) {
            const usb_device_desc_t *dd = &dinfo.dev_desc;
            ESP_LOGI(TAG,
                     "[dev] VID=0x%04X PID=0x%04X class=0x%02X sub=0x%02X proto=0x%02X speed=%s",
                     dd->idVendor, dd->idProduct, dd->bDeviceClass, dd->bDeviceSubClass,
                     dd->bDeviceProtocol,
                     (dinfo.speed == USB_SPEED_LOW  ? "LOW"  :
                      dinfo.speed == USB_SPEED_FULL ? "FULL" :
                      dinfo.speed == USB_SPEED_HIGH ? "HIGH" : "UNKNOWN"));
          } else {
            ESP_LOGW(TAG, "[dev] usb_host_device_info() failed: 0x%X", (unsigned) ie);
          }
        }
  #else
        // --- B) Tu IDF 5.4.2: usar control transfer GET_DESCRIPTOR(Device)
       if (!read_device_descriptor_ctrl_(self->client_, self->dev_handle_)) {
          ESP_LOGW(TAG, "[dev] GET_DESCRIPTOR(Device) failed or timed out");
        }
  #endif
#else
        // IDF < 5: no contemplado aquí
        if (!read_device_descriptor_ctrl_(self->dev_handle_)) {
          ESP_LOGW(TAG, "[dev] GET_DESCRIPTOR(Device) failed or timed out");
        }
#endif

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

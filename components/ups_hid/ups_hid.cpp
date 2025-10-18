#include "ups_hid.h"
#include "usb/usb_types_ch9.h"   // usb_setup_packet_t, USB_SETUP_PACKET_SIZE

namespace esphome {
namespace ups_hid {

static const char *const TAG = "ups_hid";

// --- Callback obligatorio para transfers en IDF 5.4.x (aunque no lo usemos) ---
static void ctrl_transfer_cb_(usb_transfer_t *transfer) {
  // no-op
}

// --- PLAN B: GET_DESCRIPTOR(Device) por control transfer (IDF 5.4.x y genérico) ---
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
  xfer->callback          = ctrl_transfer_cb_;  // <— IMPORTANTE: no NULL
  xfer->context           = nullptr;
  xfer->device_handle     = dev_handle;         // <— IMPORTANTE
  xfer->bEndpointAddress  = 0x00;               // EP0 (control)
  xfer->flags             = 0;

  ESP_LOGD(TAG, "[dev] ctrl submit: cb=%p dev=%p client=%p",
           (void*) xfer->callback, (void*) dev_handle, (void*) client);

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


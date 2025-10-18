#include "ups_hid.h"

namespace esphome {
namespace ups_hid {

static const char *const TAG = "ups_hid";

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
    // 4) Tarea que despacha eventos (llama handle_events periódicamente)
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
    if (err == ESP_OK || err == ESP_ERR_TIMEOUT) {
      // OK / sin eventos
    } else {
      ESP_LOGW(TAG, "[usbh_client] handle_events err=0x%X", (unsigned) err);
    }
  }
}

void UpsHid::client_callback_(const usb_host_client_event_msg_t *msg, void *arg) {
  switch (msg->event) {
    case USB_HOST_CLIENT_EVENT_NEW_DEV:
      ESP_LOGI(TAG, "[attach] NEW_DEV");
      break;
    case USB_HOST_CLIENT_EVENT_DEV_GONE:
      ESP_LOGI(TAG, "[detach] DEV_GONE");
      break;
    default:
      ESP_LOGI(TAG, "[client] event=%d", (int) msg->event);
      break;
  }
}

}  // namespace ups_hid
}  // namespace esphome

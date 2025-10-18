#include "ups_hid.h"

namespace esphome {
namespace ups_hid {

static const char *const TAG = "ups_hid";

void UpsHid::setup() {
  // 1) Instalar librería USB Host
  usb_host_config_t cfg = {
      .skip_phy_setup = false,  // usamos el PHY interno del S3
      .intr_flags = 0,
  };
  esp_err_t err = usb_host_install(&cfg);
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "USB Host Library installed.");
  } else {
    ESP_LOGE(TAG, "usb_host_install() failed: 0x%X", (unsigned)err);
    return;
  }

  // 2) Crear tarea "daemon" que atiende eventos del Host Library
  xTaskCreatePinnedToCore(UpsHid::host_daemon_task_, "usbh_daemon", 4096, nullptr, 5, nullptr, tskNO_AFFINITY);
}

void UpsHid::host_daemon_task_(void *arg) {
  uint32_t flags = 0;
  while (true) {
    esp_err_t err = usb_host_lib_handle_events(pdMS_TO_TICKS(1000), &flags);
    if (err == ESP_OK && flags) {
      ESP_LOGI(TAG, "USB Host event flags: 0x%08X", (unsigned)flags);
      flags = 0;
    } else if (err == ESP_ERR_TIMEOUT) {
      // normal, sin eventos
    } else if (err != ESP_OK) {
      ESP_LOGW(TAG, "usb_host_lib_handle_events() err=0x%X", (unsigned)err);
    }
    // (no desinstalamos el host aquí; queda residente)
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

}  // namespace ups_hid
}  // namespace esphome

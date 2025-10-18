#include "ups_hid.h"

namespace esphome {
namespace ups_hid {

static const char *const TAG = "ups_hid";

void UpsHid::setup() {
  // Deja setup vacío por ahora o con logs DEBUG si quieres.
}

void UpsHid::dump_config() {
  ESP_LOGCONFIG(TAG, "UPS HID component is configured.");
}

void UpsHid::loop() {
  if (!this->hello_logged_) {
    ESP_LOGI(TAG, "UPS HID component started (hello from ESPHome external component).");
    this->hello_logged_ = true;
  }
  // Aquí vendrá la lógica de USB Host en los siguientes pasos.
}

}  // namespace ups_hid
}  // namespace esphome

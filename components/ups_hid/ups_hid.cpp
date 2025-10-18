#include "ups_hid.h"

namespace esphome {
namespace ups_hid {

static const char *const TAG = "ups_hid";

void UpsHid::setup() {
  ESP_LOGI(TAG, "UPS HID component started (hello from ESPHome external component).");
}

void UpsHid::loop() {
  // Aquí más adelante gestionaremos eventos del USB Host y publicaremos sensores.
}

}  // namespace ups_hid
}  // namespace esphome


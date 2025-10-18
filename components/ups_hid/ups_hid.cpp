#include "ups_hid.h"

namespace esphome {
namespace ups_hid {

static const char *const TAG = "ups_hid";

void UpsHid::setup() {
  ESP_LOGI(TAG, "UPS HID component started (hello from ESPHome external component).");
}

void UpsHid::dump_config() {
  ESP_LOGCONFIG(TAG, "UPS HID component is configured.");  // <── NUEVO LOG de arranque
}

void UpsHid::loop() {
  // Pendiente: USB Host + HID
}

}  // namespace ups_hid
}  // namespace esphome

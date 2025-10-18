#pragma once
#include "esphome/core/component.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ups_hid {

class UpsHid : public PollingComponent {  // <-- PollingComponent
 public:
  void setup() override;
  void dump_config() override;
  void update() override;  // <-- en vez de loop()
 private:
  bool hello_logged_{false};
};

}  // namespace ups_hid
}  // namespace esphome

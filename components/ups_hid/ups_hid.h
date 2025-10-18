#pragma once
#include "esphome/core/component.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ups_hid {

class UpsHid : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return esphome::setup_priority::HARDWARE; }

 private:
  bool hello_logged_{false};  // <-- NUEVO
};

}  // namespace ups_hid
}  // namespace esphome

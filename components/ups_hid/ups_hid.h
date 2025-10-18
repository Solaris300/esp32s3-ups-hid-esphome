#pragma once
#include "esphome/core/component.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ups_hid {

class UpsHid : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;  // <── AÑADIDO
  float get_setup_priority() const override { return esphome::setup_priority::HARDWARE; }
};

}  // namespace ups_hid
}  // namespace esphome

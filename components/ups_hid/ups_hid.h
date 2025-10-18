#pragma once
#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "usb/usb_host.h"   // <-- Host Library base

namespace esphome {
namespace ups_hid {

class UpsHid : public PollingComponent {
 public:
  void setup() override;
  void dump_config() override;
  void update() override;

 private:
  bool hello_logged_{false};
  static void host_daemon_task_(void *arg);
  static void client_task_(void *arg);
  usb_host_client_handle_t client_{nullptr};
};

}  // namespace ups_hid
}  // namespace esphome

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
  static void client_callback_(const usb_host_client_event_msg_t *event_msg, void *arg);
  usb_host_client_handle_t client_{nullptr};

  usb_device_handle_t dev_handle_{nullptr};
  uint8_t dev_addr_{0};
};

}  // namespace ups_hid
}  // namespace esphome

#pragma once
#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "usb/usb_host.h"

namespace esphome {
namespace ups_hid {

class UpsHid : public PollingComponent {
 public:
  void setup() override;
  void dump_config() override;
  void update() override;

 private:
  bool hello_logged_{false};

  // Tareas / callback del host USB
  static void host_daemon_task_(void *arg);
  static void client_task_(void *arg);
  static void client_callback_(const usb_host_client_event_msg_t *msg, void *arg);

  // Handles básicos
  usb_host_client_handle_t client_{nullptr};
  usb_device_handle_t      dev_handle_{nullptr};
  uint8_t                  dev_addr_{0};

// Info de HID descubierta
  uint8_t  hid_if_num_{0xFF};
  uint8_t  hid_ep_in_{0};
  uint16_t hid_ep_mps_{0};
  uint8_t  hid_ep_interval_{0};
};

}  // namespace ups_hid
}  // namespace esphome

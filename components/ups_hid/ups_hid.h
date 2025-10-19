#pragma once
#include "esphome/core/component.h"
#include "esphome/core/log.h"

#include <cstring>
#include <cstdio>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "usb/usb_host.h"
#include "usb/usb_types_ch9.h"  // usb_setup_packet_t, USB_SETUP_PACKET_SIZE

namespace esphome {
namespace ups_hid {

class UpsHid : public PollingComponent {
 public:
  void setup() override;
  void dump_config() override;

  // Necesario para PollingComponent (main.cpp llamará set_update_interval)
  void update() override { /* no-op: usamos nuestras tareas */ }

  // No usamos loop(); todo va por tareas
  void loop() override {}

 private:
  // Tareas / callback
  static void host_daemon_task_(void *arg);
  static void client_task_(void *arg);
  static void client_callback_(const usb_host_client_event_msg_t *msg, void *arg);

  // Handles
  usb_host_client_handle_t client_{nullptr};
  usb_device_handle_t      dev_handle_{nullptr};
  uint8_t                  dev_addr_{0};

  // Info HID descubierta
  uint8_t  hid_if_{0xFF};        // interface number
  uint8_t  hid_ep_in_{0};        // 0x81 típico
  uint16_t hid_ep_mps_{0};
  uint8_t  hid_ep_interval_{0};

  // Control del sondeo (GET_REPORT por control EP0)
  bool      poll_enabled_{false};
  TickType_t next_poll_tick_{0};
};

}  // namespace ups_hid
}  // namespace esphome

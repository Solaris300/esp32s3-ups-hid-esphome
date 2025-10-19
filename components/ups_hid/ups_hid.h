#pragma once
#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "usb/usb_host.h"
#include "usb/usb_types_ch9.h"   // usb_setup_packet_t, USB_SETUP_PACKET_SIZE

namespace esphome {
namespace ups_hid {

class UpsHid : public PollingComponent {
 public:
  void setup() override;
  void dump_config() override;
  void update() override;

 private:
  bool hello_logged_{false};

  // Tareas / callback
  static void host_daemon_task_(void *arg);
  static void client_task_(void *arg);
  static void client_callback_(const usb_host_client_event_msg_t *msg, void *arg);

  // Handles y estado de dispositivo
  usb_host_client_handle_t client_{nullptr};
  usb_device_handle_t dev_handle_{nullptr};
  uint8_t dev_addr_{0};

  // Descubrimiento HID
  uint8_t hid_ep_in_{0};        // p.ej. 0x81
  uint16_t hid_ep_mps_{0};      // p.ej. 8
  uint8_t hid_ep_interval_{0};  // p.ej. 10 (ms, orientativo)

  // Lectura continua del endpoint
  usb_transfer_t *in_xfer_{nullptr};
  bool listening_{false};
};

}  // namespace ups_hid
}  // namespace esphome

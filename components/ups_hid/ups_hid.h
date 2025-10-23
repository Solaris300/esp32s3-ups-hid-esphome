#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <usb/usb_host.h>
#include <usb/usb_types_ch9.h>

namespace esphome {
namespace ups_hid {

class UpsHid : public PollingComponent {
 public:
  void setup() override;
  void dump_config() override;
  void update() override;  // PollingComponent pide este método; aquí no hacemos nada pesado

 private:
  // Tareas
  static void daemon_task_(void *arg);
  static void client_task_(void *arg);

  // Helpers de control transfer
  static bool dump_report_descriptor_chunked_(usb_host_client_handle_t client,
                                              usb_device_handle_t devh,
                                              uint8_t hid_if,
                                              uint16_t &total_logged);

  static bool hid_get_report_input_ctrl_(usb_host_client_handle_t client,
                                         usb_device_handle_t devh,
                                         uint8_t report_id,
                                         uint8_t *buf,
                                         int buf_len,
                                         int &out_len,
                                         uint8_t hid_if);

  // Utilidades
  static const char *xfer_status_str(usb_transfer_status_t st);
};

}  // namespace ups_hid
}  // namespace esphome

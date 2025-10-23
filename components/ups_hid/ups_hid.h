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
  // Ciclo de vida ESPHome
  void setup() override;
  void dump_config() override;
  void update() override;

 private:
  // ---- Tareas y callback del cliente USB Host ----
  static void host_daemon_task_(void *arg);
  static void client_task_(void *arg);
  static void client_callback_(const usb_host_client_event_msg_t *msg, void *arg);

  // ---- Helpers HID (control transfers) ----
  static bool read_config_descriptor_and_log_hid_(usb_host_client_handle_t client,
                                                  usb_device_handle_t dev_handle,
                                                  uint8_t &if_num, uint8_t &ep_in,
                                                  uint16_t &mps, uint8_t &interval,
                                                  uint16_t &rdesc_len);

  static bool dump_report_descriptor_chunked_(usb_host_client_handle_t client,
                                              usb_device_handle_t dev_handle,
                                              uint8_t interface_number,
                                              uint16_t &total_len_logged);

  static bool hid_get_report_input_ctrl_(usb_host_client_handle_t client,
                                         usb_device_handle_t dev_handle,
                                         uint8_t report_id,
                                         uint8_t *out_buf,
                                         int out_buf_len,
                                         int &out_len,
                                         uint8_t report_type = 0x01 /*Input*/);

  // ---- Estado del componente / USB host ----
  bool hello_logged_{false};

  usb_host_client_handle_t client_{nullptr};
  usb_device_handle_t      dev_handle_{nullptr};
  uint8_t                  dev_addr_{0};

  // HID descubierto
  uint8_t  hid_if_{0xFF};
  uint8_t  hid_ep_in_{0};
  uint16_t hid_ep_mps_{0};
  uint8_t  hid_ep_interval_{0};

  // Lectura por endpoint interrupt (no usada ahora mismo, pero dejamos por si la retomamos)
  usb_transfer_t *in_xfer_{nullptr};
  bool            listening_{false};
};

}  // namespace ups_hid
}  // namespace esphome

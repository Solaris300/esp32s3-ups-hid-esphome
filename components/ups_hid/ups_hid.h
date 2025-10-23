#pragma once
#include "esphome/core/component.h"
#include "esphome/core/log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cstring>               // memcpy, memset
#include "usb/usb_host.h"
#include "usb/usb_types_ch9.h"   // usb_setup_packet_t, USB_SETUP_PACKET_SIZE

namespace esphome {
namespace ups_hid {

class UpsHid : public PollingComponent {
 public:
  void setup() override;
  void dump_config() override;
  void update() override;  // sólo un hello una vez

 private:
  // -------- Tareas / callbacks ----------
  static void host_daemon_task_(void *arg);
  static void client_task_(void *arg);
  static void client_callback_(const usb_host_client_event_msg_t *msg, void *arg);

  // -------- Helpers de control transfers / parseo ----------
  static bool read_device_descriptor_(usb_host_client_handle_t client,
                                      usb_device_handle_t dev_handle,
                                      uint16_t &vid, uint16_t &pid);

  static bool read_config_descriptor_and_log_hid_(usb_host_client_handle_t client,
                                                  usb_device_handle_t dev_handle,
                                                  uint8_t &if_num, uint8_t &ep_in,
                                                  uint16_t &mps, uint8_t &interval,
                                                  uint16_t &rdesc_len);

  static bool dump_report_descriptor_chunked_(usb_host_client_handle_t client,
                                              usb_device_handle_t dev_handle,
                                              uint8_t if_num,
                                              uint16_t &out_total_len);

  static bool hid_get_report_input_ctrl_(usb_host_client_handle_t client,
                                         usb_device_handle_t dev_handle,
                                         uint8_t report_id,
                                         uint8_t *out_buf, int out_cap,
                                         int &out_len,
                                         uint8_t if_num);

  static void hexlog_line_(const char *prefix, const uint8_t *p, int n);

  // Opcional: armazón de decodificador mínimo
  static void parse_report_minimal_(uint8_t report_id, const uint8_t *d, int n);

 private:
  bool hello_logged_{false};

  // Handles
  usb_host_client_handle_t client_{nullptr};
  usb_device_handle_t dev_handle_{nullptr};
  uint8_t dev_addr_{0};

  // HID descubierta
  uint8_t  hid_if_{0xFF};
  uint8_t  hid_ep_in_{0};
  uint16_t hid_ep_mps_{0};
  uint8_t  hid_ep_interval_{0};
};

}  // namespace ups_hid
}  // namespace esphome

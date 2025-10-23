#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"

#include <vector>
#include <map>
#include <array>
#include <cstring>

// ESP-IDF USB Host
#include "usb/usb_host.h"
#include "usb/usb_helpers.h"
#include "usb/usb_types_ch9.h"

namespace esphome {
namespace ups_hid {

static const char *const TAG = "ups_hid";

class UpsHid : public PollingComponent {
 public:
  UpsHid();

  // esphome::Component
  void setup() override;
  void dump_config() override;

  // esphome::PollingComponent
  void update() override;

 private:
  // ====== USB Host daemon (gestiona eventos globales del host) ======
  static void daemon_task_(void *param);
  static void client_task_(void *param);

  // ====== Helpers de configuración / HID ======
  static bool read_config_descriptor_and_log_hid_(usb_host_client_handle_t client,
                                                  usb_device_handle_t devh,
                                                  uint8_t &hid_if,
                                                  uint8_t &ep_in,
                                                  uint16_t &mps_in,
                                                  uint8_t &poll_interval_ms,
                                                  uint16_t &rdesc_len);

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

  // ====== Diff logger ======
  void log_report_diff_(uint8_t report_id, const uint8_t *data, int len);

  // ====== Estado ======
  static TaskHandle_t daemon_task_handle_;
  static TaskHandle_t client_task_handle_;
  static bool host_started_;

  // Buffers para último reporte visto por ID (para “diff”)
  std::map<uint8_t, std::vector<uint8_t>> last_report_by_id_;
};

}  // namespace ups_hid
}  // namespace esphome

#pragma once
#include "esphome/core/component.h"
#include "esphome/core/log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "usb/usb_host.h"
#include "usb/usb_types_ch9.h"   // usb_setup_packet_t, USB_SETUP_PACKET_SIZE
#include <cstring>

namespace esphome {
namespace ups_hid {

class UpsHid : public Component {
 public:
  void setup() override;
  void dump_config() override;

  // usaremos set_interval desde main.cpp para el polling
  void set_interval(uint32_t interval_ms, std::function<void()> &&f) {
    this->interval_ = interval_ms;
    this->interval_cb_ = std::move(f);
  }

 private:
  // ---------- Estado ----------
  bool hello_logged_{false};

  // Handles
  usb_host_client_handle_t client_{nullptr};
  usb_device_handle_t      dev_handle_{nullptr};
  uint8_t                  dev_addr_{0};

  // HID descubierto
  uint8_t  hid_if_{0xFF};
  uint8_t  hid_ep_in_{0};
  uint16_t hid_ep_mps_{0};
  uint8_t  hid_ep_interval_{0};

  // Banderas para trabajo fuera del callback
  volatile bool probe_pending_{false};

  // Buffer de trabajo para GET_DESCRIPTOR / GET_REPORT
  // (se asignan y liberan en cada operación; no persistentes)
  // ----

  // Ring buffer simple de últimos reportes por ID (para depurar)
  static constexpr int MAX_STORED_REPORT = 8;
  uint8_t last_rep_id01_[MAX_STORED_REPORT][64];
  uint8_t last_rep_id64_[MAX_STORED_REPORT][64];
  int     last_rep_id01_count_{0};
  int     last_rep_id64_count_{0};

  // ---------- Utilidades ----------
  static void host_daemon_task_(void *arg);
  static void client_task_(void *arg);
  static void client_callback_(const usb_host_client_event_msg_t *msg, void *arg);

  static bool read_config_descriptor_and_log_hid_(
      usb_host_client_handle_t client,
      usb_device_handle_t dev_handle,
      uint8_t &if_num, uint8_t &ep_in,
      uint16_t &mps, uint8_t &interval,
      uint16_t &total_len);

  static bool dump_report_descriptor_chunked_(
      usb_host_client_handle_t client,
      usb_device_handle_t dev_handle,
      uint8_t interface_number,
      uint16_t &out_len);

  static bool hid_get_report_input_ctrl_(
      usb_host_client_handle_t client,
      usb_device_handle_t dev_handle,
      uint8_t report_id,
      uint8_t *out_buf, int out_cap, int &out_len,
      uint8_t hid_if);

  static void log_hex_line_(const char *prefix, const uint8_t *data, int len);
  void       store_last_report_(uint8_t report_id, const uint8_t *data, int len);

  // Intervalo “virtual” para permitir set_interval desde main.cpp
  uint32_t interval_{0};
  std::function<void()> interval_cb_{};
};

}  // namespace ups_hid
}  // namespace esphome

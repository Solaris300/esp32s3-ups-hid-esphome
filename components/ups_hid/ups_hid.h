#pragma once
#include "esphome/core/component.h"
#include "esphome/core/log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "usb/usb_host.h"
#include "usb/usb_types_ch9.h"  // usb_setup_packet_t, USB_SETUP_PACKET_SIZE

namespace esphome {
namespace ups_hid {

class UpsHid : public Component {
 public:
  void setup() override;
  void dump_config() override;

 private:
  // --- tareas / callbacks ---
  static void host_daemon_task_(void *arg);
  static void client_task_(void *arg);
  static void client_callback_(const usb_host_client_event_msg_t *msg, void *arg);

  // --- helpers control transfer ---
  static bool read_config_descriptor_and_log_hid_(usb_host_client_handle_t client,
                                                 usb_device_handle_t dev_handle,
                                                 uint8_t &if_num, uint8_t &ep_in,
                                                 uint16_t &mps, uint8_t &interval,
                                                 uint16_t &total_cfg_len);
  static bool dump_report_descriptor_chunked_(usb_host_client_handle_t client,
                                              usb_device_handle_t dev_handle,
                                              uint8_t if_num,
                                              uint16_t &out_total_len);
  static bool hid_get_report_input_ctrl_(usb_host_client_handle_t client,
                                         usb_device_handle_t dev_handle,
                                         uint8_t report_id,
                                         uint8_t *out_buf, int out_buf_len,
                                         int &out_len,
                                         uint8_t interface_number);

  // --- estado USB host/cliente/dispositivo ---
  usb_host_client_handle_t client_{nullptr};
  usb_device_handle_t      dev_handle_{nullptr};
  uint8_t                  dev_addr_{0};

  // --- estado HID descubierto ---
  uint8_t  hid_if_{0xFF};
  uint8_t  hid_ep_in_{0};
  uint16_t hid_ep_mps_{0};
  uint8_t  hid_ep_interval_{0};

  // --- flags para la client_task_ ---
  static volatile bool g_probe_pending_;
  static volatile bool g_polling_enabled_;

  // --- buffers de comparación (diff) por report id ---
  // Soportamos hasta 3 IDs conocidos (0x01, 0x64, 0x66). Se puede ampliar fácil.
  static constexpr int MAX_REPORT_LEN = 64;

  uint8_t last_rep_01_[MAX_REPORT_LEN] = {0};
  int     last_rep_01_len_ = 0;
  bool    have_last_01_ = false;

  uint8_t last_rep_64_[MAX_REPORT_LEN] = {0};
  int     last_rep_64_len_ = 0;
  bool    have_last_64_ = false;

  uint8_t last_rep_66_[MAX_REPORT_LEN] = {0};
  int     last_rep_66_len_ = 0;
  bool    have_last_66_ = false;

  // --- utilidades internas ---
  void log_diff_report_(uint8_t rep_id, const uint8_t *data, int len);
  void log_hex_prefix_(const char *prefix, const uint8_t *data, int len, int max_bytes);

  // --- throttling de logs de diff ---
  uint32_t last_diff_log_ms_ = 0;
};

}  // namespace ups_hid
}  // namespace esphome

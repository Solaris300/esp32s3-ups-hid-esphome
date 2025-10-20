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
  // ---------- Tareas / callbacks ----------
  static void host_daemon_task_(void *arg);
  static void client_task_(void *arg);
  static void client_callback_(const usb_host_client_event_msg_t *msg, void *arg);

  // ---------- Helpers de control transfer ----------
  static bool get_config_header_(usb_host_client_handle_t client, usb_device_handle_t dev, uint8_t *cfg_hdr_out_9);
  static bool get_full_config_(usb_host_client_handle_t client, usb_device_handle_t dev, uint8_t *buf, int buf_len, int &out_len);
  static bool read_config_descriptor_and_log_hid_(usb_host_client_handle_t client, usb_device_handle_t dev_handle,
                                                  uint8_t &if_num, uint8_t &ep_in,
                                                  uint16_t &mps, uint8_t &interval, uint16_t &rdesc_len);
  static bool get_report_descriptor_(usb_host_client_handle_t client, usb_device_handle_t dev_handle, uint8_t if_num,
                                     uint8_t *buf, int buf_len, int &out_len);
  static void dump_report_descriptor_(const uint8_t *buf, int len);

  // ¡Cambio aquí!: pasamos también el if_num para no usar `this` en estática
  static bool hid_get_report_input_ctrl_(usb_host_client_handle_t client, usb_device_handle_t dev_handle,
                                         uint8_t if_num, uint8_t report_id,
                                         uint8_t *out_buf, int out_cap, int &out_len);

  // ---------- Estado ----------
  usb_host_client_handle_t client_{nullptr};
  usb_device_handle_t      dev_handle_{nullptr};
  uint8_t                  dev_addr_{0};

  // Descubrimiento HID
  uint8_t  hid_if_{0xFF};
  uint8_t  hid_ep_in_{0};
  uint16_t hid_ep_mps_{0};
  uint8_t  hid_ep_interval_{0};
  uint16_t hid_report_desc_len_{0};

  // Flags de trabajo (se resuelven en client_task_)
  bool probe_pending_{false};  // tras NEW_DEV
  bool rdump_done_{false};     // ya se volcó el Report Descriptor en esta conexión

  // Sondeo periódico
  uint32_t last_poll_ms_{0};

  // (Próximo paso: sensores y parser de reports)
};

}  // namespace ups_hid
}  // namespace esphome

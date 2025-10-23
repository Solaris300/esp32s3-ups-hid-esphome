#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"

#include <cstring>
#include <cstdint>

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
  // ---- Tareas / callbacks ----
  static void host_daemon_task_(void *arg);
  static void client_task_(void *arg);
  static void client_callback_(const usb_host_client_event_msg_t *msg, void *arg);

  // ---- Helpers de control transfer (sincrónicos) ----
  static bool read_config_descriptor_and_log_hid_(usb_host_client_handle_t client,
                                                  usb_device_handle_t dev_handle,
                                                  uint8_t &if_num, uint8_t &ep_in,
                                                  uint16_t &mps, uint8_t &interval,
                                                  uint16_t &rdesc_len);

  static bool dump_report_descriptor_chunked_(usb_host_client_handle_t client,
                                              usb_device_handle_t dev_handle,
                                              uint8_t if_num, uint16_t &out_len);

  static bool hid_get_report_input_ctrl_(usb_host_client_handle_t client,
                                         usb_device_handle_t dev_handle,
                                         uint8_t if_num,
                                         uint8_t report_id,
                                         uint8_t *out_buf, int out_buf_sz, int &out_len,
                                         uint8_t report_type = 0x01 /*INPUT*/);

  // ---- Estado ----
  usb_host_client_handle_t client_{nullptr};
  usb_device_handle_t      dev_handle_{nullptr};
  uint8_t                  dev_addr_{0};

  // HID descubierta
  uint8_t  hid_if_{0xFF};           // interfaz HID
  uint8_t  hid_ep_in_{0};           // endpoint IN interrupt (por si más adelante usamos INTR)
  uint16_t hid_ep_mps_{0};          // max packet size
  uint8_t  hid_ep_interval_{0};     // intervalo eP IN (ms aprox)
  uint16_t hid_rdesc_len_{0};       // longitud del Report Descriptor (aprox/esperada)

  // Control de descubrimiento y sondeo
  bool probe_pending_{false};       // solicitado desde el callback NEW_DEV
  uint8_t poll_idx_{0};             // índice para rotar report IDs

  // Utilidad de log
  static void log_hex_line_(const char *tag, const uint8_t *data, int len);
};

}  // namespace ups_hid
}  // namespace esphome

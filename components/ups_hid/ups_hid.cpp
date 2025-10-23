#include "ups_hid.h"
#include "esphome/core/application.h"

#include <cstdio>
#include <cinttypes>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

namespace esphome {
namespace ups_hid {

TaskHandle_t UpsHid::daemon_task_handle_{nullptr};
TaskHandle_t UpsHid::client_task_handle_{nullptr};
bool UpsHid::host_started_{false};

UpsHid::UpsHid() : PollingComponent(1000) {}  // 1s por defecto

void UpsHid::setup() {
  // Arrancamos el host una sola vez
  if (!host_started_) {
    usb_host_config_t cfg = {
        .intr_flags = 0,
    };
    esp_err_t err = usb_host_install(&cfg);
    if (err == ESP_OK) {
      host_started_ = true;
      xTaskCreatePinnedToCore(&UpsHid::daemon_task_, "usbh_daemon", 4096, this, 5, &daemon_task_handle_, tskNO_AFFINITY);
      xTaskCreatePinnedToCore(&UpsHid::client_task_, "usbh_client", 8192, this, 5, &client_task_handle_, tskNO_AFFINITY);
    } else {
      ESP_LOGE(TAG, "usb_host_install failed: %d", (int) err);
    }
  }
}

void UpsHid::dump_config() {
  ESP_LOGCONFIG(TAG, "UPS HID:");
  ESP_LOGCONFIG(TAG, "  Polling interval: %" PRIu32 " ms", this->get_update_interval());
}

void UpsHid::update() {
  // No hacemos nada aquí: todo va en la tarea cliente con su propio ritmo.
}

// ===================== UTILIDADES =====================

static const char *xfer_status_str(usb_transfer_status_t st) {
  switch (st) {
    case USB_TRANSFER_STATUS_COMPLETED: return "COMPLETED";
    case USB_TRANSFER_STATUS_NO_DEVICE: return "NO_DEVICE";
    case USB_TRANSFER_STATUS_ERROR:     return "ERROR";
    case USB_TRANSFER_STATUS_STALL:     return "STALL";
    case USB_TRANSFER_STATUS_BUSY:      return "BUSY";
    case USB_TRANSFER_STATUS_CANCELED:  return "CANCELED";
    default:                            return "UNKNOWN";
  }
}

// ====== Diff Logger ======
void UpsHid::log_report_diff_(uint8_t report_id, const uint8_t *data, int len) {
  auto &prev = last_report_by_id_[report_id];

  // Si nunca hemos visto este ID, lo guardamos y volcamos la línea completa como antes
  if (prev.empty()) {
    prev.assign(data, data + len);

    // Línea clásica (completa) para el primer reporte de cada ID
    // Para no inundar, imprimimos en bloques de 32 bytes
    char line[256];
    int off = 0;
    while (off < len) {
      int chunk = std::min(32, len - off);
      int pos = 0;
      pos += snprintf(line + pos, sizeof(line) - pos, "[poll] GET_REPORT id=0x%02X len=%d data=", report_id, len);
      for (int i = 0; i < chunk; i++) {
        pos += snprintf(line + pos, sizeof(line) - pos, "%02X ", data[off + i]);
        if (pos >= (int) sizeof(line) - 4) break;
      }
      ESP_LOGI(TAG, "%s", line);
      off += chunk;
    }
    return;
  }

  // Compara y construye una cadena compacta marcando diferencias
  // Formato: “idx: old->new”
  char buf[512];
  int pos = snprintf(buf, sizeof(buf), "[poll] DIFF id=0x%02X changed:", report_id);
  bool any = false;

  int max_cmp = std::min<int>(prev.size(), len);
  for (int i = 0; i < max_cmp; i++) {
    if (prev[i] != data[i]) {
      any = true;
      pos += snprintf(buf + pos, sizeof(buf) - pos, " %d:%02X->%02X", i, prev[i], data[i]);
      if (pos > (int) sizeof(buf) - 16) break;
    }
  }
  if (len != (int) prev.size()) {
    any = true;
    pos += snprintf(buf + pos, sizeof(buf) - pos, " (len %zu->%d)", prev.size(), len);
  }

  if (any) {
    ESP_LOGI(TAG, "%s", buf);
    // Guardamos la nueva referencia
    prev.assign(data, data + len);
  } else {
    // Nada cambió; para que sepas que hubo lectura pero igual a la previa:
    ESP_LOGV(TAG, "[poll] DIFF id=0x%02X no changes", report_id);
  }
}

// ====== Lectura de configuración / HID ======

bool UpsHid::read_config_descriptor_and_log_hid_(usb_host_client_handle_t client,
                                                 usb_device_handle_t devh,
                                                 uint8_t &hid_if,
                                                 uint8_t &ep_in,
                                                 uint16_t &mps_in,
                                                 uint8_t &poll_interval_ms,
                                                 uint16_t &rdesc_len) {
  const usb_device_desc_t *ddesc;
  if (usb_host_get_device_descriptor(devh, &ddesc) != ESP_OK) {
    ESP_LOGE(TAG, "[devdesc] failed");
    return false;
  }
  ESP_LOGI(TAG, "[usbh_client]: [devdesc] VID=0x%04X PID=0x%04X", ddesc->idVendor, ddesc->idProduct);

  const usb_config_desc_t *cfg;
  if (usb_host_get_active_config_descriptor(devh, &cfg) != ESP_OK) {
    ESP_LOGE(TAG, "[cfg] get active config failed");
    return false;
  }

  hid_if = 0xFF;
  ep_in = 0;
  mps_in = 0;
  poll_interval_ms = 0;
  rdesc_len = 0;

  const uint8_t *p = (const uint8_t *) cfg;
  const uint8_t *end = p + cfg->wTotalLength;

  while (p + sizeof(usb_desc_hdr_t) <= end) {
    auto *hdr = (const usb_desc_hdr_t *) p;
    if (hdr->bLength == 0) break;

    if (hdr->bDescriptorType == USB_B_DESCRIPTOR_TYPE_INTERFACE) {
      auto *ifd = (const usb_intf_desc_t *) p;
      if (ifd->bInterfaceClass == USB_CLASS_HID && ifd->bInterfaceSubClass == 0x01 /*Boot*/ && ifd->bInterfaceProtocol == 0x00 /*None*/) {
        hid_if = ifd->bInterfaceNumber;
        ESP_LOGI(TAG, "[usbh_client]: [cfg] HID IF=%u class=0x%02X sub=0x%02X proto=0x%02X",
                 (unsigned) ifd->bInterfaceNumber, ifd->bInterfaceClass, ifd->bInterfaceSubClass, ifd->bInterfaceProtocol);
      }
    } else if (hdr->bDescriptorType == USB_B_DESCRIPTOR_TYPE_ENDPOINT) {
      auto *ep = (const usb_ep_desc_t *) p;
      if ((ep->bEndpointAddress & USB_B_ENDPOINT_ADDRESS_EP_DIR_MASK) == USB_B_ENDPOINT_ADDRESS_EP_DIR_IN) {
        ep_in = ep->bEndpointAddress;
        mps_in = ep->wMaxPacketSize;
        poll_interval_ms = ep->bInterval;
        ESP_LOGI(TAG, "[usbh_client]: [cfg] HID endpoint IN=0x%02X MPS=%u interval=%u ms",
                 ep_in, (unsigned) mps_in, (unsigned) poll_interval_ms);
      }
    } else if (hdr->bDescriptorType == USB_B_DESCRIPTOR_TYPE_HID) {
      // HID descriptor (para longitud de Report Descriptor)
      if (hdr->bLength >= 9) {
        // bLength 9, bytes 7..8 => wDescriptorLength
        rdesc_len = *((uint16_t *)(p + 7));
        ESP_LOGI(TAG, "[usbh_client]: [cfg] Report Descriptor length=%u bytes", (unsigned) rdesc_len);
      }
    }

    p += hdr->bLength;
  }

  if (hid_if == 0xFF || ep_in == 0) {
    ESP_LOGE(TAG, "[cfg] HID interface/endpoint not found");
    return false;
  }
  ESP_LOGI(TAG, "[usbh_client]: [cfg] ready: IF=%u EP=0x%02X MPS=%u interval=%u",
           (unsigned) hid_if, ep_in, (unsigned) mps_in, (unsigned) poll_interval_ms);

  return true;
}

bool UpsHid::dump_report_descriptor_chunked_(usb_host_client_handle_t client,
                                             usb_device_handle_t devh,
                                             uint8_t hid_if,
                                             uint16_t &total_logged) {
  total_logged = 0;

  // GET_DESCRIPTOR (Report)
  const int CHUNK = 32;
  std::vector<uint8_t> tmp(CHUNK);

  // Vamos leyendo en tramos con control transfer
  // Nota: algunos dispositivos aceptan offset en wValue / wIndex combinando,
  // aquí hacemos intentos consecutivos hasta que falle por STALL/NO_DEVICE.
  for (int off = 0; off < 2048; off += CHUNK) {
    usb_setup_packet_t setup = {};
    setup.bmRequestType = USB_BM_REQUEST_TYPE_DIR_IN | USB_BM_REQUEST_TYPE_TYPE_STANDARD | USB_BM_REQUEST_TYPE_RECIP_INTERFACE;
    setup.bRequest = USB_B_REQUEST_GET_DESCRIPTOR;
    // HID Report descriptor = (HID_REPORT << 8) | 0
    setup.wValue = (0x22 << 8) | 0x00;
    setup.wIndex = hid_if;
    setup.wLength = CHUNK;

    usb_transfer_t *xfer = nullptr;
    if (usb_host_transfer_alloc(CHUNK, 0, &xfer) != ESP_OK) break;
    memcpy(xfer->data_buffer, &setup, sizeof(setup));
    xfer->num_bytes = sizeof(setup);
    xfer->callback = nullptr;
    xfer->context  = nullptr;

    esp_err_t err = usb_host_transfer_submit_control(devh, xfer);
    if (err != ESP_OK) {
      usb_host_transfer_free(xfer);
      break;
    }
    // Espera activa simple
    while (xfer->status == USB_TRANSFER_STATUS_BUSY) { vTaskDelay(pdMS_TO_TICKS(1)); }

    if (xfer->status != USB_TRANSFER_STATUS_COMPLETED) {
      // Terminamos cuando ya no hay más (algunos controladores devuelven STALL al sobre-pedir)
      usb_host_transfer_free(xfer);
      break;
    }

    int got = xfer->actual_num_bytes;
    if (got <= 0) {
      usb_host_transfer_free(xfer);
      break;
    }

    // Logueamos en líneas de 16 bytes
    if (off == 0) {
      ESP_LOGI(TAG, "[usbh_client]: [rdesc] len (chunked) ~>= %d bytes", got * 4); // aproximado para avisarte
    }
    int ptr = 0;
    while (ptr < got) {
      int n = std::min(16, got - ptr);
      char line[128]; int pos = 0;
      pos += snprintf(line + pos, sizeof(line) - pos, "[usbh_client]: [rdesc] ");
      for (int i = 0; i < n; i++) pos += snprintf(line + pos, sizeof(line) - pos, "%02X ", xfer->data_buffer[ptr + i]);
      ESP_LOGI(TAG, "%s", line);
      ptr += n;
    }

    total_logged += got;
    usb_host_transfer_free(xfer);

    // Evita bloquear demasiado el logger
    vTaskDelay(pdMS_TO_TICKS(1));
    if (got < CHUNK) break;  // última porción
  }

  ESP_LOGI(TAG, "[usbh_client]: [rdesc] len=%u bytes", (unsigned) total_logged * 2);
  return (total_logged > 0);
}

bool UpsHid::hid_get_report_input_ctrl_(usb_host_client_handle_t client,
                                        usb_device_handle_t devh,
                                        uint8_t report_id,
                                        uint8_t *buf,
                                        int buf_len,
                                        int &out_len,
                                        uint8_t hid_if) {
  out_len = 0;

  // HID GET_REPORT(Input)
  usb_setup_packet_t setup = {};
  setup.bmRequestType = USB_BM_REQUEST_TYPE_DIR_IN | USB_BM_REQUEST_TYPE_TYPE_CLASS | USB_BM_REQUEST_TYPE_RECIP_INTERFACE;
  setup.bRequest = 0x01; // GET_REPORT
  setup.wValue   = (1 /*Input*/ << 8) | report_id;
  setup.wIndex   = hid_if;
  setup.wLength  = buf_len;

  usb_transfer_t *xfer = nullptr;
  if (usb_host_transfer_alloc(buf_len, 0, &xfer) != ESP_OK) {
    return false;
  }
  // Cargar setup packet en el buffer interno
  memcpy(xfer->data_buffer, &setup, sizeof(setup));
  xfer->num_bytes = sizeof(setup);

  esp_err_t err = usb_host_transfer_submit_control(devh, xfer);
  if (err != ESP_OK) {
    usb_host_transfer_free(xfer);
    return false;
  }

  while (xfer->status == USB_TRANSFER_STATUS_BUSY) { vTaskDelay(pdMS_TO_TICKS(1)); }

  bool ok = (xfer->status == USB_TRANSFER_STATUS_COMPLETED);
  if (ok) {
    // El payload llega en xfer->data_buffer (después del setup), pero
    // usb_host_transfer de IDF ya nos deja el “actual_num_bytes”
    int n = xfer->actual_num_bytes;
    if (n > buf_len) n = buf_len;
    if (n > 0) {
      memcpy(buf, xfer->data_buffer, n);
      out_len = n;
    }
  } else {
    ESP_LOGW(TAG, "[poll] GET_REPORT id=0x%02X status=%s", report_id, xfer_status_str(xfer->status));
  }

  usb_host_transfer_free(xfer);
  return ok;
}

// ====== Tareas ======

void UpsHid::daemon_task_(void *param) {
  ESP_LOGI(TAG, "[usbh_daemon] USB Host daemon started");
  while (true) {
    uint32_t events = 0;
    esp_err_t err = usb_host_lib_handle_events(1000 /*ms*/, &events);
    if (err == ESP_OK) {
      if (events) {
        ESP_LOGI(TAG, "[usbh_daemon] USB Host event flags: 0x%08" PRIx32, events);
      }
    } else if (err != ESP_ERR_TIMEOUT) {
      ESP_LOGW(TAG, "[usbh_daemon] handle_events err=%d", (int) err);
    }
  }
}

void UpsHid::client_task_(void *param) {
  auto *self = static_cast<UpsHid *>(param);

  usb_host_client_handle_t client = nullptr;
  usb_host_client_config_t client_cfg = {};
  client_cfg.is_synchronous = false;
  client_cfg.max_num_event_msg = 5;
  client_cfg.async = { .client_event_callback = nullptr, .callback_arg = nullptr };

  if (usb_host_client_register(&client_cfg, &client) != ESP_OK) {
    ESP_LOGE(TAG, "[usbh_client] register failed");
    vTaskDelete(nullptr);
    return;
  }

  while (true) {
    usb_host_dev_hdl_t devh = nullptr;
    usb_host_client_handle_events(client, 1000);

    // Espera a que haya un dispositivo (bloqueante de cortesía)
    if (usb_host_device_open(client, 1 /* address autodetect not provided here */, &devh) != ESP_OK) {
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    ESP_LOGI(TAG, "[usbh_client]: [attach] NEW_DEV (opened)");

    uint8_t hid_if = 0, ep_in = 0, poll_ms = 10;
    uint16_t mps_in = 8, rdesc_len = 0;
    if (!read_config_descriptor_and_log_hid_(client, devh, hid_if, ep_in, mps_in, poll_ms, rdesc_len)) {
      usb_host_device_close(client, devh);
      continue;
    }

    // Intenta volcar (troceado) el Report Descriptor para no saturar logger
    uint16_t logged = 0;
    dump_report_descriptor_chunked_(client, devh, hid_if, logged);

    // ===== Bucle de “poll” con GET_REPORT por IDs conocidos =====
    const uint8_t report_ids[] = {0x01, 0x64, 0x66};
    std::array<uint8_t, 64> buf{};
    for (;;) {
      bool still_ok = true;
      for (uint8_t rid : report_ids) {
        int out_len = 0;
        if (!hid_get_report_input_ctrl_(client, devh, rid, buf.data(), buf.size(), out_len, hid_if)) {
          still_ok = false;
          break;
        }
        if (out_len > 0) {
          // <<<— AQUÍ el nuevo logger “diff” —>>>
          self->log_report_diff_(rid, buf.data(), out_len);
        }
        vTaskDelay(pdMS_TO_TICKS(5));  // pequeño respiro entre IDs
      }
      if (!still_ok) break;
      vTaskDelay(pdMS_TO_TICKS(1000)); // ritmo ~1s total, similar a tus logs
    }

    ESP_LOGI(TAG, "[usbh_client]: [detach] DEV_GONE");
    usb_host_device_close(client, devh);
    // Limpia referencias “diff” para no mezclar sesiones
    self->last_report_by_id_.clear();
  }
}

}  // namespace ups_hid
}  // namespace esphome

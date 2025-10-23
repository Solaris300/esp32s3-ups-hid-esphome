#include "ups_hid.h"
#include "esphome/core/log.h"

#include <cstring>
#include <algorithm>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <usb/usb_host.h>
#include <usb/usb_types_ch9.h>

namespace esphome {
namespace ups_hid {

static const char *const TAG = "ups_hid";

void UpsHid::setup() {
  // Arrancamos las dos tareas: demonio (gestiona la capa host) y cliente (abre dispositivo y hace polls)
  xTaskCreatePinnedToCore(daemon_task_, "usbh_daemon", 4096, nullptr, 5, nullptr, tskNO_AFFINITY);
  xTaskCreatePinnedToCore(client_task_, "usbh_client", 8192, nullptr, 5, nullptr, tskNO_AFFINITY);
}

void UpsHid::dump_config() {
  ESP_LOGCONFIG(TAG, "UPS HID component iniciado (modo experimento GET_REPORT por control).");
}

void UpsHid::update() {
  // No hacemos nada aquí, el trabajo lo hace la tarea client en segundo plano
}

// ---------------------- helpers ----------------------

const char *UpsHid::xfer_status_str(usb_transfer_status_t st) {
  switch (st) {
    case USB_TRANSFER_STATUS_COMPLETED: return "COMPLETED";
    case USB_TRANSFER_STATUS_NO_DEVICE: return "NO_DEVICE";
    case USB_TRANSFER_STATUS_ERROR:     return "ERROR";
    case USB_TRANSFER_STATUS_STALL:     return "STALL";
    case USB_TRANSFER_STATUS_CANCELED:  return "CANCELED";
    default:                            return "UNKNOWN";
  }
}

bool UpsHid::hid_get_report_input_ctrl_(usb_host_client_handle_t client,
                                        usb_device_handle_t devh,
                                        uint8_t report_id,
                                        uint8_t *buf,
                                        int buf_len,
                                        int &out_len,
                                        uint8_t hid_if) {
  out_len = 0;

  usb_setup_packet_t setup = {};
  setup.bmRequestType = USB_BM_REQUEST_TYPE_DIR_IN | USB_BM_REQUEST_TYPE_TYPE_CLASS | USB_BM_REQUEST_TYPE_RECIP_INTERFACE;
  setup.bRequest = 0x01;                 // GET_REPORT
  setup.wValue   = (1 /*Input*/ << 8) | report_id;
  setup.wIndex   = hid_if;
  setup.wLength  = buf_len;

  usb_transfer_t *xfer = nullptr;
  // Espacio para setup + datos
  if (usb_host_transfer_alloc(sizeof(usb_setup_packet_t) + buf_len, 0, &xfer) != ESP_OK) {
    ESP_LOGW(TAG, "[poll] alloc falló");
    return false;
  }

  // Rellenar control (CLAVE)
  usb_host_transfer_fill_control(xfer, &setup, nullptr, buf_len);
  xfer->device_handle = devh;
  xfer->callback = nullptr;
  xfer->context  = nullptr;

  esp_err_t err = usb_host_transfer_submit_control(client, xfer);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "[poll] submit_control err=%d", (int) err);
    usb_host_transfer_free(xfer);
    return false;
  }

  bool done = false;
  for (int i = 0; i < 60; i++) {  // ~300ms
    usb_host_client_handle_events(client, 5);
    if (xfer->status == USB_TRANSFER_STATUS_COMPLETED ||
        xfer->status == USB_TRANSFER_STATUS_STALL ||
        xfer->status == USB_TRANSFER_STATUS_ERROR ||
        xfer->status == USB_TRANSFER_STATUS_NO_DEVICE ||
        xfer->status == USB_TRANSFER_STATUS_CANCELED) {
      done = true;
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }

  bool ok = done && (xfer->status == USB_TRANSFER_STATUS_COMPLETED);
  if (ok) {
    // Datos vienen tras el setup
    int n = xfer->actual_num_bytes;
    if (n > buf_len) n = buf_len;
    const uint8_t *data = xfer->data_buffer + sizeof(usb_setup_packet_t);
    memcpy(buf, data, n);
    out_len = n;
  } else {
    ESP_LOGW(TAG, "[poll] GET_REPORT id=0x%02X status=%s bytes=%d",
             report_id, xfer_status_str(xfer->status), (int) xfer->actual_num_bytes);
  }

  usb_host_transfer_free(xfer);
  return ok;
}

bool UpsHid::dump_report_descriptor_chunked_(usb_host_client_handle_t client,
                                             usb_device_handle_t devh,
                                             uint8_t hid_if,
                                             uint16_t &total_logged) {
  total_logged = 0;
  const int CHUNK = 32;

  for (int off = 0; off < 2048; off += CHUNK) {
    usb_setup_packet_t setup = {};
    setup.bmRequestType = USB_BM_REQUEST_TYPE_DIR_IN | USB_BM_REQUEST_TYPE_TYPE_STANDARD | USB_BM_REQUEST_TYPE_RECIP_INTERFACE;
    setup.bRequest = USB_B_REQUEST_GET_DESCRIPTOR;
    setup.wValue   = (0x22 /*Report Descriptor*/ << 8) | 0x00;
    setup.wIndex   = hid_if;
    setup.wLength  = CHUNK;

    usb_transfer_t *xfer = nullptr;
    if (usb_host_transfer_alloc(sizeof(usb_setup_packet_t) + CHUNK, 0, &xfer) != ESP_OK) break;

    usb_host_transfer_fill_control(xfer, &setup, nullptr, CHUNK);
    xfer->device_handle = devh;

    if (usb_host_transfer_submit_control(client, xfer) != ESP_OK) {
      usb_host_transfer_free(xfer);
      break;
    }

    bool done = false;
    for (int i = 0; i < 40; i++) {
      usb_host_client_handle_events(client, 5);
      if (xfer->status == USB_TRANSFER_STATUS_COMPLETED ||
          xfer->status == USB_TRANSFER_STATUS_STALL ||
          xfer->status == USB_TRANSFER_STATUS_ERROR ||
          xfer->status == USB_TRANSFER_STATUS_NO_DEVICE ||
          xfer->status == USB_TRANSFER_STATUS_CANCELED) {
        done = true; break;
      }
      vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (!done || xfer->status != USB_TRANSFER_STATUS_COMPLETED) {
      usb_host_transfer_free(xfer);
      break;
    }

    int got = xfer->actual_num_bytes;
    const uint8_t *data = xfer->data_buffer + sizeof(usb_setup_packet_t);

    if (got > 0) {
      // Log en líneas de 16 bytes
      int ptr = 0;
      while (ptr < got) {
        int n = std::min(16, got - ptr);
        char line[128]; int pos = 0;
        pos += snprintf(line + pos, sizeof(line) - pos, "[usbh_client]: [rdesc] ");
        for (int i = 0; i < n; i++) pos += snprintf(line + pos, sizeof(line) - pos, "%02X ", data[ptr + i]);
        ESP_LOGI(TAG, "%s", line);
        ptr += n;
      }
    }

    total_logged += got;
    usb_host_transfer_free(xfer);
    if (got < CHUNK) break;  // Último trozo
  }

  ESP_LOGI(TAG, "[usbh_client]: [rdesc] len~=%u bytes", (unsigned) total_logged);
  return (total_logged > 0);
}

// ---------------------- tareas ----------------------

void UpsHid::daemon_task_(void *arg) {
  ESP_LOGI(TAG, "[usbh_daemon] arrancando USB Host...");
  usb_host_config_t host_cfg = {};
  host_cfg.intr_flags = ESP_INTR_FLAG_LEVEL1;

  if (usb_host_install(&host_cfg) != ESP_OK) {
    ESP_LOGE(TAG, "[usbh_daemon] usb_host_install falló");
    vTaskDelete(nullptr);
    return;
  }

  // Manejamos eventos globales (desconexiones, etc.)
  while (true) {
    uint32_t events = 0;
    esp_err_t err = usb_host_lib_handle_events(portMAX_DELAY, &events);
    if (err == ESP_OK) {
      ESP_LOGI(TAG, "[usbh_daemon] USB Host event flags: 0x%08X", (unsigned) events);
      if (events & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
        // Sin clientes: podemos liberar si quisiéramos
      }
      if (events & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
        // Todos los recursos liberados
      }
    }
  }
}

void UpsHid::client_task_(void *arg) {
  usb_host_client_config_t client_cfg = {};
  client_cfg.is_synchronous = false;
  client_cfg.max_num_event_msg = 32;
  client_cfg.async.client_event_callback = nullptr;
  client_cfg.async.callback_arg = nullptr;

  usb_host_client_handle_t client = nullptr;
  if (usb_host_client_register(&client_cfg, &client) != ESP_OK) {
    ESP_LOGE(TAG, "[usbh_client] client_register falló");
    vTaskDelete(nullptr);
    return;
  }

  // Intentamos abrir por direcciones típicas 1..5 (según logs usabas 1 ó 2)
  usb_device_handle_t devh = nullptr;
  for (;;) {
    for (int addr = 1; addr <= 5 && devh == nullptr; addr++) {
      if (usb_host_device_open(client, addr, &devh) == ESP_OK) {
        ESP_LOGI(TAG, "[usbh_client]: [attach] NEW_DEV addr=%d (opened)", addr);
        break;
      }
    }
    if (devh) break;
    // Espera a que haya algo conectado
    usb_host_client_handle_events(client, 100);
    vTaskDelay(pdMS_TO_TICKS(200));
  }

  // Log (opcional) del descriptor de dispositivo (VID/PID)
  {
    usb_device_info_t info = {};
    if (usb_host_device_info(devh, &info) == ESP_OK) {
      ESP_LOGI(TAG, "[usbh_client]: [devdesc] VID=0x%04X PID=0x%04X", (unsigned) info.dev_desc.idVendor, (unsigned) info.dev_desc.idProduct);
    }
  }

  // Asumimos HID en interfaz 0
  const uint8_t hid_if = 0;

  // Intento de volcado del Report Descriptor (en trozos)
  uint16_t logged = 0;
  dump_report_descriptor_chunked_(client, devh, hid_if, logged);

  ESP_LOGI(TAG, "[usbh_client]: [cfg] ready: IF=%d EP=0x%02X MPS=%d interval=%d",
           0, 0x81, 8, 10);

  // Bucle de polls de GET_REPORT (IDs 0x01, 0x64, 0x66) por control
  const uint8_t report_ids[] = {0x01, 0x64, 0x66};
  uint8_t buf[64];
  for (;;) {
    for (uint8_t rid : report_ids) {
      int out_len = 0;
      if (hid_get_report_input_ctrl_(client, devh, rid, buf, sizeof(buf), out_len, hid_if)) {
        // Formato de log similar al tuyo
        char line[256]; int pos = 0;
        pos += snprintf(line + pos, sizeof(line) - pos, "[poll] GET_REPORT id=0x%02X len=%d data=", rid, out_len);
        // Si el primer byte es Report ID, mejor mostrarlo tal cual recibimos
        for (int i = 0; i < out_len; i++) {
          if (pos + 3 >= (int) sizeof(line)) break;
          pos += snprintf(line + pos, sizeof(line) - pos, "%02X ", buf[i]);
        }
        ESP_LOGI(TAG, "%s", line);
      }
      // pequeño respiro entre IDs
      vTaskDelay(pdMS_TO_TICKS(20));
    }
    // ciclo cada ~1s
    vTaskDelay(pdMS_TO_TICKS(900));
    // Procesamos eventos pendientes
    usb_host_client_handle_events(client, 5);
  }

  // (Nunca llegamos aquí en esta demo; si cerráramos, habría que hacer:
  // usb_host_device_close(client, devh); usb_host_client_deregister(client);)
}

}  // namespace ups_hid
}  // namespace esphome

#include "ups_hid.h"
#include <cstring>
#include "usb/usb_types_ch9.h"   // usb_setup_packet_t, USB_SETUP_PACKET_SIZE
#include "esp_timer.h"           // esp_timer_get_time()

namespace esphome {
namespace ups_hid {

static const char *const TAG = "ups_hid";

// -----------------------------------------------------
// Estado simple para orquestar pasos sin tocar el .h
// -----------------------------------------------------
static UpsHid *g_self = nullptr;
static volatile bool g_probe_pending   = false;  // tras NEW_DEV: descubrir interfaz/endpoint
static volatile bool g_dump_rdesc_once = false;  // tras descubrir IF: leer Report Descriptor una vez
static volatile bool g_poll_reports    = false;  // tras todo listo: empezar GET_REPORT periódico
static uint8_t g_if_num = 0;                     // interfaz HID detectada

// -----------------------------------------------------
// Utilidades: alloc + submit + wait (CTRL xfer)
// -----------------------------------------------------
static bool submit_ctrl_and_wait_(usb_host_client_handle_t client, usb_transfer_t *xfer, uint32_t timeout_ms) {
  if (!client || !xfer) return false;
  esp_err_t e = usb_host_transfer_submit_control(client, xfer);
  if (e != ESP_OK) {
    ESP_LOGW(TAG, "[ctrl] submit failed: 0x%X", (unsigned) e);
    return false;
  }
  const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
  while (xTaskGetTickCount() < deadline) {
    (void) usb_host_client_handle_events(client, pdMS_TO_TICKS(10));
    if (xfer->status == USB_TRANSFER_STATUS_COMPLETED ||
        xfer->status == USB_TRANSFER_STATUS_ERROR ||
        xfer->status == USB_TRANSFER_STATUS_STALL ||
        xfer->status == USB_TRANSFER_STATUS_NO_DEVICE ||
        xfer->status == USB_TRANSFER_STATUS_CANCELED) {
      break;
    }
  }
  return xfer->status == USB_TRANSFER_STATUS_COMPLETED;
}

static bool alloc_setup_(int payload_len, usb_device_handle_t dev, usb_transfer_t **out_xfer) {
  *out_xfer = nullptr;
  const int total = USB_SETUP_PACKET_SIZE + payload_len;
  if (usb_host_transfer_alloc(total, 0, out_xfer) != ESP_OK) return false;
  usb_transfer_t *x = *out_xfer;
  x->num_bytes = total;
  x->callback = nullptr;
  x->context  = nullptr;
  x->device_handle    = dev;
  x->bEndpointAddress = 0x00; // EP0
  x->flags            = 0;
  return true;
}

// -----------------------------------------------------
// Lee Configuration Descriptor -> detecta IF HID y EP IN
// -----------------------------------------------------
static bool read_config_descriptor_and_log_hid_(usb_host_client_handle_t client,
                                                usb_device_handle_t dev_handle,
                                                uint8_t &if_num, uint8_t &ep_in,
                                                uint16_t &mps, uint8_t &interval) {
  if_num = 0; ep_in = 0; mps = 0; interval = 0;
  if (!client || !dev_handle) return false;

  // A) Header (9 bytes)
  usb_transfer_t *xhdr = nullptr;
  if (!alloc_setup_(9, dev_handle, &xhdr)) {
    ESP_LOGW(TAG, "[cfg] alloc header failed");
    return false;
  }
  {
    auto *sh = (usb_setup_packet_t *) xhdr->data_buffer;
    sh->bmRequestType = 0x80; sh->bRequest = 0x06; // GET_DESCRIPTOR
    sh->wValue        = (uint16_t)((2 << 8) | 0);  // CONFIGURATION
    sh->wIndex        = 0;
    sh->wLength       = 9;
  }
  if (!submit_ctrl_and_wait_(client, xhdr, 1500)) {
    ESP_LOGW(TAG, "[cfg] header transfer failed status=0x%X", (unsigned) xhdr->status);
    usb_host_transfer_free(xhdr);
    return false;
  }
  const uint8_t *cfg_hdr = xhdr->data_buffer + USB_SETUP_PACKET_SIZE;
  if (cfg_hdr[1] != 2 || cfg_hdr[0] < 9) {
    ESP_LOGW(TAG, "[cfg] invalid header bType=%u bLen=%u", cfg_hdr[1], cfg_hdr[0]);
    usb_host_transfer_free(xhdr);
    return false;
  }
  uint16_t wTotalLength = (uint16_t)(cfg_hdr[2] | (cfg_hdr[3] << 8));
  usb_host_transfer_free(xhdr);

  // B) Descriptor completo
  usb_transfer_t *xfull = nullptr;
  if (!alloc_setup_(wTotalLength, dev_handle, &xfull)) {
    ESP_LOGW(TAG, "[cfg] alloc full failed");
    return false;
  }
  {
    auto *sf = (usb_setup_packet_t *) xfull->data_buffer;
    sf->bmRequestType = 0x80; sf->bRequest = 0x06; // GET_DESCRIPTOR
    sf->wValue        = (uint16_t)((2 << 8) | 0);  // CONFIGURATION
    sf->wIndex        = 0;
    sf->wLength       = wTotalLength;
  }
  if (!submit_ctrl_and_wait_(client, xfull, 2000)) {
    ESP_LOGW(TAG, "[cfg] full transfer failed status=0x%X", (unsigned) xfull->status);
    usb_host_transfer_free(xfull);
    return false;
  }

  // C) Parseo: INTERFACE HID + ENDPOINT IN interrupt
  const uint8_t *p   = xfull->data_buffer + USB_SETUP_PACKET_SIZE;
  const uint8_t *end = p + wTotalLength;
  int hid_if = -1;
  while (p + 2 <= end && p[0] >= 2 && p + p[0] <= end) {
    uint8_t len = p[0], type = p[1];
    if (type == 4 && len >= 9) { // INTERFACE
      uint8_t bInterfaceNumber = p[2];
      uint8_t bClass = p[5], bSub = p[6], bProto = p[7];
      if (bClass == 0x03 && hid_if < 0) {
        hid_if = (int) bInterfaceNumber;
        ESP_LOGI(TAG, "[cfg] HID IF=%d class=0x%02X sub=0x%02X proto=0x%02X",
                 (int)bInterfaceNumber, bClass, bSub, bProto);
      }
    } else if (type == 5 && len >= 7 && hid_if >= 0 && ep_in == 0) { // ENDPOINT
      uint8_t bEndpointAddress = p[2];
      bool is_in  = (bEndpointAddress & 0x80) != 0;
      bool is_intr= ((p[3] & 0x03) == 3);
      if (is_in && is_intr) {
        ep_in   = bEndpointAddress;
        mps     = (uint16_t)(p[4] | (p[5] << 8));
        interval= p[6];
      }
    }
    p += len;
  }

  usb_host_transfer_free(xfull);

  if (hid_if >= 0 && ep_in != 0) {
    if_num = (uint8_t) hid_if;
    ESP_LOGI(TAG, "[cfg] HID endpoint IN=0x%02X MPS=%u interval=%u ms",
             ep_in, (unsigned) mps, (unsigned) interval);
    return true;
  }
  ESP_LOGW(TAG, "[cfg] No se encontró interfaz HID o endpoint IN.");
  return false;
}

// -----------------------------------------------------
// Lee HID Report Descriptor (tipo 0x22) en IF = if_num
// -----------------------------------------------------
static bool hid_get_report_descriptor_(usb_host_client_handle_t client,
                                       usb_device_handle_t dev,
                                       uint8_t if_num,
                                       uint8_t *out_buf, int max_len, int &got_len) {
  got_len = 0;
  if (!client || !dev || !out_buf || max_len <= 0) return false;

  // Intento con 512 bytes (suele bastar para HID UPS)
  const int req_len = (max_len < 512) ? max_len : 512;
  usb_transfer_t *x = nullptr;
  if (!alloc_setup_(req_len, dev, &x)) {
    ESP_LOGW(TAG, "[rdesc] alloc failed");
    return false;
  }
  {
    auto *s = (usb_setup_packet_t *) x->data_buffer;
    s->bmRequestType = 0x81;                 // IN | Class | Interface
    s->bRequest      = 0x06;                 // GET_DESCRIPTOR
    s->wValue        = (uint16_t)((0x22 << 8) | 0); // REPORT descriptor
    s->wIndex        = if_num;               // interfaz HID
    s->wLength       = req_len;
  }
  bool ok = submit_ctrl_and_wait_(client, x, 2000);
  if (!ok) {
    ESP_LOGW(TAG, "[rdesc] transfer failed status=0x%X", (unsigned) x->status);
    usb_host_transfer_free(x);
    return false;
  }
  // Copia el payload
  int n = x->actual_num_bytes;
  if (n > req_len) n = req_len;
  if (n > max_len) n = max_len;
  if (n > 0) {
    std::memcpy(out_buf, x->data_buffer + USB_SETUP_PACKET_SIZE, n);
    got_len = n;
  }
  usb_host_transfer_free(x);
  return got_len > 0;
}

// -----------------------------------------------------
// GET_REPORT (Input report) por control (polling)
// -----------------------------------------------------
static bool hid_get_input_report_(usb_host_client_handle_t client,
                                  usb_device_handle_t dev,
                                  uint8_t if_num,
                                  uint8_t report_id,
                                  uint8_t *out_buf, int max_len, int &got_len) {
  got_len = 0;
  if (!client || !dev || !out_buf || max_len <= 0) return false;

  usb_transfer_t *x = nullptr;
  if (!alloc_setup_(max_len, dev, &x)) {
    ESP_LOGW(TAG, "[poll] alloc failed");
    return false;
  }
  {
    auto *s = (usb_setup_packet_t *) x->data_buffer;
    s->bmRequestType = 0xA1;                // IN | Class | Interface
    s->bRequest      = 0x01;                // GET_REPORT
    // Alto: TYPE(1=Input), Bajo: REPORT ID (0 si no usa IDs)
    s->wValue        = (uint16_t)((0x01 << 8) | report_id);
    s->wIndex        = if_num;
    s->wLength       = max_len;
  }
  bool ok = submit_ctrl_and_wait_(client, x, 1200);
  if (!ok) {
    ESP_LOGW(TAG, "[poll] transfer failed status=0x%X", (unsigned) x->status);
    usb_host_transfer_free(x);
    return false;
  }
  int n = x->actual_num_bytes;
  if (n > max_len) n = max_len;
  if (n > 0) {
    std::memcpy(out_buf, x->data_buffer + USB_SETUP_PACKET_SIZE, n);
    got_len = n;
  }
  usb_host_transfer_free(x);
  return got_len > 0;
}

// =====================================================
// Métodos de la clase
// =====================================================

void UpsHid::setup() {
  // 1) Instalar librería USB Host
  usb_host_config_t cfg = {.skip_phy_setup = false, .intr_flags = 0};
  esp_err_t err = usb_host_install(&cfg);
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "USB Host Library installed.");
  } else {
    ESP_LOGE(TAG, "usb_host_install() failed: 0x%X", (unsigned) err);
    return;
  }

  // 2) Tarea daemon de la librería
  xTaskCreatePinnedToCore(UpsHid::host_daemon_task_, "usbh_daemon",
                          4096, nullptr, 5, nullptr, tskNO_AFFINITY);

  // 3) Registrar cliente asíncrono con callback
  usb_host_client_config_t client_cfg = {
      .is_synchronous = false,
      .max_num_event_msg = 8,
      .async = {
          .client_event_callback = UpsHid::client_callback_,
          .callback_arg = this,
      },
  };
  err = usb_host_client_register(&client_cfg, &this->client_);
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "USB Host client registered.");
    // 4) Tarea que despacha eventos
    xTaskCreatePinnedToCore(UpsHid::client_task_, "usbh_client",
                            4096, this, 5, nullptr, tskNO_AFFINITY);
  } else {
    ESP_LOGE(TAG, "usb_host_client_register() failed: 0x%X", (unsigned) err);
    return;
  }

  g_self = this;
}

void UpsHid::dump_config() {
  ESP_LOGCONFIG(TAG, "UPS HID component is configured.");
  ESP_LOGI(TAG, "UPS Host init step ready (no HID yet).");
}

void UpsHid::update() {
  if (!this->hello_logged_) {
    ESP_LOGI(TAG, "UPS HID component started (hello from ESPHome external component).");
    this->hello_logged_ = true;
  }
}

// =====================================================
// Funciones estáticas (tareas/callback)
// =====================================================

void UpsHid::host_daemon_task_(void *arg) {
  uint32_t flags = 0;
  while (true) {
    esp_err_t err = usb_host_lib_handle_events(pdMS_TO_TICKS(1000), &flags);
    if (err == ESP_OK) {
      if (flags) {
        ESP_LOGI(TAG, "[usbh_daemon] USB Host event flags: 0x%08X", (unsigned) flags);
        flags = 0;
      }
    } else if (err == ESP_ERR_TIMEOUT) {
      // sin eventos
    } else {
      ESP_LOGW(TAG, "[usbh_daemon] handle_events err=0x%X", (unsigned) err);
    }
  }
}

void UpsHid::client_task_(void *arg) {
  auto *self = static_cast<UpsHid *>(arg);
  if (!self || !self->client_) {
    ESP_LOGE(TAG, "[usbh_client] No client handle.");
    vTaskDelete(nullptr);
    return;
  }

  // temporizador para polling sencillo
  int64_t last_poll_us = 0;

  while (true) {
    esp_err_t err = usb_host_client_handle_events(self->client_, pdMS_TO_TICKS(100));
    if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
      ESP_LOGW(TAG, "[usbh_client] handle_events err=0x%X", (unsigned) err);
    }

    // 1) Descubrimiento HID (fuera del callback)
    if (g_probe_pending && self->dev_handle_ != nullptr) {
      uint8_t ifn, ep; uint16_t mps; uint8_t itv;
      if (read_config_descriptor_and_log_hid_(self->client_, self->dev_handle_, ifn, ep, mps, itv)) {
        g_if_num            = ifn;
        self->hid_ep_in_    = ep;
        self->hid_ep_mps_   = mps;
        self->hid_ep_interval_ = itv;
        ESP_LOGI(TAG, "[cfg] ready: IF=%u EP=0x%02X MPS=%u interval=%u",
                 (unsigned) g_if_num, (unsigned) ep, (unsigned) mps, (unsigned) itv);
        g_dump_rdesc_once = true; // siguiente paso: leer Report Descriptor
      }
      g_probe_pending = false;
    }

    // 2) Dump único del HID Report Descriptor
    if (g_dump_rdesc_once && self->dev_handle_ != nullptr) {
      uint8_t buf[512];
      int n = 0;
      if (hid_get_report_descriptor_(self->client_, self->dev_handle_, g_if_num, buf, sizeof(buf), n)) {
        // Log en hex acotado
        char line[3*32+1];
        int pos = 0;
        ESP_LOGI(TAG, "[rdesc] len=%d bytes", n);
        for (int i = 0; i < n; i++) {
          pos += snprintf(line + (pos>= (int)sizeof(line) ? (int)sizeof(line)-1 : pos),
                          sizeof(line) - pos, "%02X%s", buf[i],
                          ((i % 32)==31 || i==n-1) ? "" : " ");
          if ((i % 32) == 31 || i == n - 1) {
            ESP_LOGI(TAG, "[rdesc] %s", line);
            pos = 0;
          }
        }
      } else {
        ESP_LOGW(TAG, "[rdesc] GET_DESCRIPTOR(Report) falló");
      }
      // activar polling de informes de entrada por control
      g_poll_reports    = true;
      g_dump_rdesc_once = false;
      last_poll_us = esp_timer_get_time();
    }

    // 3) Polling cada ~1000 ms del Input Report (por control)
    if (g_poll_reports && self->dev_handle_ != nullptr) {
      int64_t now = esp_timer_get_time();
      if (now - last_poll_us >= 1000 * 1000) {
        last_poll_us = now;
        uint8_t rep[64];
        int got = 0;
        if (hid_get_input_report_(self->client_, self->dev_handle_, g_if_num, /*report_id*/ 0, rep, sizeof(rep), got) && got > 0) {
          // Log corto en hex
          int max_log = got < 32 ? got : 32;
          char buf[3*32 + 1]; int k = 0;
          for (int i = 0; i < max_log; i++) {
            k += snprintf(buf + k, sizeof(buf) - k, "%02X%s", rep[i], (i + 1 < max_log ? " " : ""));
            if (k >= (int)sizeof(buf)) break;
          }
          ESP_LOGI(TAG, "[poll] GET_REPORT len=%d data=%s%s", got, buf, (got > max_log ? " ..." : ""));
          // TODO: aquí es donde mapearemos campos (voltaje, %bat, etc.) según tu descriptor
        }
      }
    }
  }
}

void UpsHid::client_callback_(const usb_host_client_event_msg_t *msg, void *arg) {
  auto *self = static_cast<UpsHid *>(arg);
  if (!self) return;

  switch (msg->event) {
    case USB_HOST_CLIENT_EVENT_NEW_DEV: {
      // Abrir el dispositivo y guardar addr/handle
      esp_err_t e = usb_host_device_open(self->client_, msg->new_dev.address, &self->dev_handle_);
      if (e == ESP_OK) {
        self->dev_addr_ = msg->new_dev.address;
        ESP_LOGI(TAG, "[attach] NEW_DEV addr=%u (opened)", (unsigned) self->dev_addr_);
        // Lanzamos descubrimiento fuera del callback
        g_probe_pending   = true;
        g_dump_rdesc_once = false;
        g_poll_reports    = false;
      } else {
        ESP_LOGW(TAG, "[attach] NEW_DEV addr=%u but open failed: 0x%X",
                 (unsigned) msg->new_dev.address, (unsigned) e);
      }
      break;
    }
    case USB_HOST_CLIENT_EVENT_DEV_GONE: {
      // Reset estados
      g_probe_pending   = false;
      g_dump_rdesc_once = false;
      g_poll_reports    = false;
      if (self->dev_handle_ != nullptr) {
        usb_host_device_close(self->client_, self->dev_handle_);
        self->dev_handle_ = nullptr;
      }
      self->dev_addr_ = 0;
      self->hid_ep_in_ = 0;
      ESP_LOGI(TAG, "[detach] DEV_GONE");
      break;
    }
    default:
      ESP_LOGI(TAG, "[client] event=%d", (int) msg->event);
      break;
  }
}

}  // namespace ups_hid
}  // namespace esphome

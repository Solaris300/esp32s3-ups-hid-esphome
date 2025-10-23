#include "ups_hid.h"
#include "usb/usb_types_ch9.h"  // usb_setup_packet_t, USB_SETUP_PACKET_SIZE
#include <cstring>              // memcpy, memset, snprintf

namespace esphome {
namespace ups_hid {

static const char *const TAG = "ups_hid";

// -----------------------------------------------------
// Estado simple para coordinar con el callback del cliente
// -----------------------------------------------------
static UpsHid *g_self = nullptr;
static volatile bool g_probe_pending = false;    // lanzar descubrimiento fuera del callback

// callback no-op para transfers de control
static void ctrl_transfer_cb_(usb_transfer_t *transfer) { (void) transfer; }

// -----------------------------------------------------
// Helper: lee Configuration Descriptor, saca IF HID y EP IN interrupt
//         y obtiene la longitud del Report Descriptor (wDescriptorLength)
// -----------------------------------------------------
bool UpsHid::read_config_descriptor_and_log_hid_(usb_host_client_handle_t client,
                                                 usb_device_handle_t dev_handle,
                                                 uint8_t &if_num, uint8_t &ep_in,
                                                 uint16_t &mps, uint8_t &interval,
                                                 uint16_t &rdesc_len) {
  if_num = 0xFF; ep_in = 0; mps = 0; interval = 0; rdesc_len = 0;
  if (!client || !dev_handle) return false;

  // A) Leer cabecera (9 bytes) del descriptor de configuración
  const int hdr_len = 9;
  const int hdr_tot = USB_SETUP_PACKET_SIZE + hdr_len;

  usb_transfer_t *xhdr = nullptr;
  if (usb_host_transfer_alloc(hdr_tot, 0, &xhdr) != ESP_OK) {
    ESP_LOGW(TAG, "[cfg] transfer_alloc header failed");
    return false;
  }

  usb_setup_packet_t *sh = reinterpret_cast<usb_setup_packet_t *>(xhdr->data_buffer);
  sh->bmRequestType = 0x80;  // IN, Standard, Device
  sh->bRequest      = 0x06;  // GET_DESCRIPTOR
  sh->wValue        = static_cast<uint16_t>((2 << 8) | 0);  // CONFIGURATION (type=2, index=0)
  sh->wIndex        = 0;
  sh->wLength       = hdr_len;

  xhdr->num_bytes         = hdr_tot;
  xhdr->callback          = ctrl_transfer_cb_;
  xhdr->context           = nullptr;
  xhdr->device_handle     = dev_handle;
  xhdr->bEndpointAddress  = 0x00;
  xhdr->flags             = 0;

  if (usb_host_transfer_submit_control(client, xhdr) != ESP_OK) {
    ESP_LOGW(TAG, "[cfg] submit header failed");
    usb_host_transfer_free(xhdr);
    return false;
  }

  {
    TickType_t dl = xTaskGetTickCount() + pdMS_TO_TICKS(1500);
    while (xTaskGetTickCount() < dl) {
      (void) usb_host_client_handle_events(client, pdMS_TO_TICKS(10));
      // Espera hasta estado terminal (compatibilidad IDF)
      if (xhdr->status == USB_TRANSFER_STATUS_COMPLETED ||
          xhdr->status == USB_TRANSFER_STATUS_ERROR ||
          xhdr->status == USB_TRANSFER_STATUS_STALL ||
          xhdr->status == USB_TRANSFER_STATUS_NO_DEVICE ||
          xhdr->status == USB_TRANSFER_STATUS_CANCELED) {
        break;
      }
    }
  }

  if (xhdr->status != USB_TRANSFER_STATUS_COMPLETED) {
    ESP_LOGW(TAG, "[cfg] header status=0x%X", (unsigned) xhdr->status);
    usb_host_transfer_free(xhdr);
    return false;
  }

  const uint8_t *cfg_hdr = xhdr->data_buffer + USB_SETUP_PACKET_SIZE;
  if (cfg_hdr[1] != 2 || cfg_hdr[0] < 9) {
    ESP_LOGW(TAG, "[cfg] invalid header bType=%u bLen=%u", cfg_hdr[1], cfg_hdr[0]);
    usb_host_transfer_free(xhdr);
    return false;
  }
  uint16_t wTotalLength = static_cast<uint16_t>(cfg_hdr[2] | (cfg_hdr[3] << 8));
  usb_host_transfer_free(xhdr);

  // B) Leer descriptor completo
  int payload = wTotalLength;
  int total   = USB_SETUP_PACKET_SIZE + payload;

  usb_transfer_t *xf = nullptr;
  if (usb_host_transfer_alloc(total, 0, &xf) != ESP_OK) {
    ESP_LOGW(TAG, "[cfg] transfer_alloc full failed");
    return false;
  }

  usb_setup_packet_t *sf = reinterpret_cast<usb_setup_packet_t *>(xf->data_buffer);
  sf->bmRequestType = 0x80;  // IN, Standard, Device
  sf->bRequest      = 0x06;  // GET_DESCRIPTOR
  sf->wValue        = static_cast<uint16_t>((2 << 8) | 0);  // CONFIGURATION
  sf->wIndex        = 0;
  sf->wLength       = payload;

  xf->num_bytes         = total;
  xf->callback          = ctrl_transfer_cb_;
  xf->context           = nullptr;
  xf->device_handle     = dev_handle;
  xf->bEndpointAddress  = 0x00;
  xf->flags             = 0;

  if (usb_host_transfer_submit_control(client, xf) != ESP_OK) {
    ESP_LOGW(TAG, "[cfg] submit full failed");
    usb_host_transfer_free(xf);
    return false;
  }

  {
    TickType_t dl = xTaskGetTickCount() + pdMS_TO_TICKS(1500);
    while (xTaskGetTickCount() < dl) {
      (void) usb_host_client_handle_events(client, pdMS_TO_TICKS(10));
      // Espera hasta estado terminal
      if (xf->status == USB_TRANSFER_STATUS_COMPLETED ||
          xf->status == USB_TRANSFER_STATUS_ERROR ||
          xf->status == USB_TRANSFER_STATUS_STALL ||
          xf->status == USB_TRANSFER_STATUS_NO_DEVICE ||
          xf->status == USB_TRANSFER_STATUS_CANCELED) {
        break;
      }
    }
  }

  if (xf->status != USB_TRANSFER_STATUS_COMPLETED) {
    ESP_LOGW(TAG, "[cfg] full status=0x%X", (unsigned) xf->status);
    usb_host_transfer_free(xf);
    return false;
  }

  // C) Parseo del descriptor: localizar INTERFACE HID + su ENDPOINT IN interrupt
  const uint8_t *p   = xf->data_buffer + USB_SETUP_PACKET_SIZE;
  const uint8_t *end = p + payload;

  int hid_if = -1;
  uint16_t hid_rdesc_len = 0;

  while (p + 2 <= end && p[0] >= 2 && p + p[0] <= end) {
    uint8_t len  = p[0];
    uint8_t type = p[1];

    if (type == 4 && len >= 9) {  // INTERFACE
      uint8_t bInterfaceNumber = p[2];
      uint8_t bClass = p[5], bSub = p[6], bProto = p[7];
      if (bClass == 0x03 && hid_if < 0) {
        hid_if = bInterfaceNumber;
        ESP_LOGI(TAG, "[cfg] HID IF=%d class=0x%02X sub=0x%02X proto=0x%02X",
                 (int) bInterfaceNumber, bClass, bSub, bProto);
      }
    } else if (type == 0x21 && len >= 9 && hid_if >= 0 && hid_rdesc_len == 0) {
      // HID descriptor: bLength, bDescriptorType(0x21), bcdHID(2), bCountry, bNumDesc,
      //                {bDescType(0x22), wDescLength(2)}, ...
      uint8_t bNumDesc = p[5];
      const uint8_t *q = p + 6;
      for (uint8_t i = 0; i < bNumDesc; i++) {
        if (q + 3 <= p + len) {
          uint8_t  dt  = q[0];
          uint16_t dln = (uint16_t)(q[1] | (q[2] << 8));
          if (dt == 0x22 && dln > 0) {  // Report Descriptor
            hid_rdesc_len = dln;
            break;
          }
        }
        q += 3;
      }
    } else if (type == 5 && len >= 7 && hid_if >= 0 && ep_in == 0) {  // ENDPOINT
      uint8_t bEndpointAddress = p[2];
      bool is_in   = (bEndpointAddress & 0x80) != 0;
      bool is_intr = ((p[3] & 0x03) == 3);
      if (is_in && is_intr) {
        ep_in   = bEndpointAddress;
        mps     = (uint16_t)(p[4] | (p[5] << 8));
        interval= p[6];
      }
    }
    p += len;
  }

  usb_host_transfer_free(xf);

  if (hid_if >= 0 && ep_in != 0) {
    if_num    = (uint8_t) hid_if;
    rdesc_len = hid_rdesc_len;
    ESP_LOGI(TAG, "[cfg] HID endpoint IN=0x%02X MPS=%u interval=%u ms",
             ep_in, (unsigned) mps, (unsigned) interval);
    return true;
  } else {
    ESP_LOGW(TAG, "[cfg] No se encontró interfaz HID o endpoint IN.");
    return false;
  }
}

// -----------------------------------------------------
// Helper: vuelca el Report Descriptor en bloques de 32 bytes
// -----------------------------------------------------
bool UpsHid::dump_report_descriptor_chunked_(usb_host_client_handle_t client,
                                             usb_device_handle_t dev_handle,
                                             uint8_t interface_number,
                                             uint16_t &total_len_logged) {
  total_len_logged = 0;
  if (!client || !dev_handle) return false;

  const int CHUNK = 64;  // pedimos de 64 en 64 para no saturar el logger
  const int MAX_ONELINE = 32;

  // Para conocer longitud total, primero pedimos 1 byte y vemos cuánto devuelve el dispositivo;
  // como ya traemos rdesc_len de la config, esto se usa solo como safety si fuese 0.
  // Aquí simplemente descargamos hasta que el dispositivo deje de darnos más (o nos cansemos).
  int offset = 0;
  for (;;) {
    int ask = CHUNK;
    // SETUP: GET_DESCRIPTOR (REPORT=0x22) sobre la interface
    int tot = USB_SETUP_PACKET_SIZE + ask;
    usb_transfer_t *x = nullptr;
    if (usb_host_transfer_alloc(tot, 0, &x) != ESP_OK) {
      ESP_LOGW(TAG, "[rdesc] alloc failed");
      return (total_len_logged > 0);
    }

    usb_setup_packet_t *s = reinterpret_cast<usb_setup_packet_t *>(x->data_buffer);
    s->bmRequestType = 0x81;  // IN, Standard, Interface
    s->bRequest      = 0x06;  // GET_DESCRIPTOR
    s->wValue        = static_cast<uint16_t>((0x22 << 8) | 0);  // REPORT descriptor
    s->wIndex        = interface_number;  // interface #
    s->wLength       = ask;

    x->num_bytes         = tot;
    x->callback          = ctrl_transfer_cb_;
    x->context           = nullptr;
    x->device_handle     = dev_handle;
    x->bEndpointAddress  = 0x00;
    x->flags             = 0;

    if (usb_host_transfer_submit_control(client, x) != ESP_OK) {
      ESP_LOGW(TAG, "[rdesc] submit failed");
      usb_host_transfer_free(x);
      return (total_len_logged > 0);
    }

    {
      TickType_t dl = xTaskGetTickCount() + pdMS_TO_TICKS(1500);
      while (xTaskGetTickCount() < dl) {
        (void) usb_host_client_handle_events(client, pdMS_TO_TICKS(10));
        // Espera hasta estado terminal
        if (x->status == USB_TRANSFER_STATUS_COMPLETED ||
            x->status == USB_TRANSFER_STATUS_ERROR ||
            x->status == USB_TRANSFER_STATUS_STALL ||
            x->status == USB_TRANSFER_STATUS_NO_DEVICE ||
            x->status == USB_TRANSFER_STATUS_CANCELED) {
          break;
        }
      }
    }

    if (x->status != USB_TRANSFER_STATUS_COMPLETED) {
      usb_host_transfer_free(x);
      break;  // fin
    }

    const uint8_t *d = x->data_buffer + USB_SETUP_PACKET_SIZE;
    int got = x->actual_num_bytes;
    if (got <= 0) {
      usb_host_transfer_free(x);
      break;
    }

    if (offset == 0) {
      ESP_LOGI(TAG, "[rdesc] len=%d bytes", got);
    }

    // Logueamos en líneas de 16-32 bytes para no bloquear el logger demasiado
    int idx = 0;
    while (idx < got) {
      int n = (got - idx > MAX_ONELINE) ? MAX_ONELINE : (got - idx);
      char line[3 * MAX_ONELINE + 1];
      int k = 0;
      for (int i = 0; i < n; i++) {
        k += snprintf(line + k, sizeof(line) - k, "%02X%s", d[idx + i], (i + 1 < n ? " " : ""));
        if (k >= (int) sizeof(line)) break;
      }
      ESP_LOGI(TAG, "[rdesc] %s", line);
      idx += n;
      total_len_logged += n;
    }

    offset += got;
    usb_host_transfer_free(x);

    // Si el dispositivo devuelve menos que lo pedido, asumimos fin
    if (got < ask) break;

    // Seguridad: no volcar infinito
    if (total_len_logged > 2048) break;
  }

  return (total_len_logged > 0);
}

// -----------------------------------------------------
// Helper: HID GET_REPORT (por control) de tipo INPUT
// -----------------------------------------------------
bool UpsHid::hid_get_report_input_ctrl_(usb_host_client_handle_t client,
                                        usb_device_handle_t dev_handle,
                                        uint8_t report_id,
                                        uint8_t *out_buf,
                                        int out_buf_len,
                                        int &out_len,
                                        uint8_t report_type) {
  out_len = 0;
  if (!client || !dev_handle || !out_buf || out_buf_len <= 0) return false;

  int tot = USB_SETUP_PACKET_SIZE + out_buf_len;

  usb_transfer_t *x = nullptr;
  if (usb_host_transfer_alloc(tot, 0, &x) != ESP_OK) {
    ESP_LOGW(TAG, "[poll] alloc failed");
    return false;
  }

  usb_setup_packet_t *s = reinterpret_cast<usb_setup_packet_t *>(x->data_buffer);
  s->bmRequestType = 0xA1;  // IN, Class, Interface
  s->bRequest      = 0x01;  // GET_REPORT
  s->wValue        = static_cast<uint16_t>(((report_type & 0x03) << 8) | (report_id & 0xFF));
  s->wIndex        = g_self ? g_self->hid_if_ : 0;  // interface actual
  s->wLength       = out_buf_len;

  x->num_bytes         = tot;
  x->callback          = ctrl_transfer_cb_;
  x->context           = nullptr;
  x->device_handle     = dev_handle;
  x->bEndpointAddress  = 0x00;
  x->flags             = 0;

  if (usb_host_transfer_submit_control(client, x) != ESP_OK) {
    ESP_LOGW(TAG, "[poll] submit failed");
    usb_host_transfer_free(x);
    return false;
  }

  {
    TickType_t dl = xTaskGetTickCount() + pdMS_TO_TICKS(1500);
    while (xTaskGetTickCount() < dl) {
      (void) usb_host_client_handle_events(client, pdMS_TO_TICKS(10));
      // Espera hasta estado terminal
      if (x->status == USB_TRANSFER_STATUS_COMPLETED ||
          x->status == USB_TRANSFER_STATUS_ERROR ||
          x->status == USB_TRANSFER_STATUS_STALL ||
          x->status == USB_TRANSFER_STATUS_NO_DEVICE ||
          x->status == USB_TRANSFER_STATUS_CANCELED) {
        break;
      }
    }
  }

  bool ok = false;
  if (x->status == USB_TRANSFER_STATUS_COMPLETED) {
    int got = x->actual_num_bytes;
    if (got > 0) {
      const uint8_t *d = x->data_buffer + USB_SETUP_PACKET_SIZE;
      int n = (got > out_buf_len) ? out_buf_len : got;
      memcpy(out_buf, d, n);
      out_len = n;
      ok = true;
    }
  }
  usb_host_transfer_free(x);
  return ok;
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
    // 4) Tarea que despacha eventos + polling
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
}

void UpsHid::update() {
  // No hacemos nada periódico aquí; el polling se hace en client_task_.
  if (!this->hello_logged_) {
    ESP_LOGI(TAG, "UPS HID component ready.");
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

  TickType_t last_poll = xTaskGetTickCount();

  while (true) {
    esp_err_t err = usb_host_client_handle_events(self->client_, pdMS_TO_TICKS(100));
    if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
      ESP_LOGW(TAG, "[usbh_client] handle_events err=0x%X", (unsigned) err);
    }

    // 1) Descubrimiento HID (fuera del callback)
    if (g_probe_pending && self->dev_handle_ != nullptr) {
      uint8_t if_num, ep; uint16_t mps; uint8_t itv; uint16_t rdlen;
      if (read_config_descriptor_and_log_hid_(self->client_, self->dev_handle_,
                                              if_num, ep, mps, itv, rdlen)) {
        self->hid_if_         = if_num;
        self->hid_ep_in_      = ep;
        self->hid_ep_mps_     = mps;
        self->hid_ep_interval_= itv;

        ESP_LOGI(TAG, "[cfg] ready: IF=%u EP=0x%02X MPS=%u interval=%u",
                 (unsigned) self->hid_if_, (unsigned) self->hid_ep_in_,
                 (unsigned) self->hid_ep_mps_, (unsigned) self->hid_ep_interval_);

        // Volcar Report Descriptor (troceado)
        uint16_t dumped = 0;
        (void) dump_report_descriptor_chunked_(self->client_, self->dev_handle_, self->hid_if_, dumped);
      }
      g_probe_pending = false;
    }

    // 2) Polling por control GET_REPORT (cada ~1000 ms)
    TickType_t now = xTaskGetTickCount();
    if (self->dev_handle_ != nullptr &&
        self->hid_if_ != 0xFF &&
        (now - last_poll) >= pdMS_TO_TICKS(1000)) {
      last_poll = now;

      uint8_t buf[64];
      int outlen = 0;

      // ids que ya viste responder: 0x01, 0x64, 0x66
      const uint8_t report_ids[] = {0x01, 0x64, 0x66};
      for (uint8_t rid : report_ids) {
        if (hid_get_report_input_ctrl_(self->client_, self->dev_handle_, rid,
                                       buf, sizeof(buf), outlen, 0x01 /*Input*/)) {
          // Log breve en hex
          int max_log = (outlen > 64) ? 64 : outlen;
          char line[3 * 64 + 1];
          int k = 0;
          for (int i = 0; i < max_log; i++) {
            k += snprintf(line + k, sizeof(line) - k, "%02X%s", buf[i], (i + 1 < max_log ? " " : ""));
            if (k >= (int) sizeof(line)) break;
          }
          ESP_LOGI(TAG, "[poll] GET_REPORT id=0x%02X len=%d data=%s%s",
                   rid, outlen, line, (outlen > max_log ? " ..." : ""));
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
        // lanzamos descubrimiento fuera del callback
        g_probe_pending = true;
      } else {
        ESP_LOGW(TAG, "[attach] NEW_DEV addr=%u but open failed: 0x%X",
                 (unsigned) msg->new_dev.address, (unsigned) e);
      }
      break;
    }
    case USB_HOST_CLIENT_EVENT_DEV_GONE: {
      // Parar lectura si estaba activa
      self->listening_ = false;

      if (self->in_xfer_) {
        // Si alguna vez armamos IN por interrupt, la liberaríamos fuera del ISR
        usb_host_transfer_free(self->in_xfer_);
        self->in_xfer_ = nullptr;
      }

      if (self->dev_handle_ != nullptr) {
        usb_host_device_close(self->client_, self->dev_handle_);
        self->dev_handle_ = nullptr;
      }
      self->dev_addr_       = 0;
      self->hid_if_         = 0xFF;
      self->hid_ep_in_      = 0;
      self->hid_ep_mps_     = 0;
      self->hid_ep_interval_= 0;

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

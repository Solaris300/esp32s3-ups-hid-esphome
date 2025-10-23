#include "ups_hid.h"

namespace esphome {
namespace ups_hid {

static const char *const TAG = "ups_hid";

// -----------------------------------------------------
// Estado simple + banderas para trabajo fuera de callback
// -----------------------------------------------------
static UpsHid *g_self = nullptr;
static volatile bool g_probe_pending = false;   // hacer discovery fuera del callback

// No-op para control transfers (evitamos warnings)
static void ctrl_transfer_cb_(usb_transfer_t *x) { (void)x; }

// -----------------------------------------------------
// Util: log en líneas cortas de hex
// -----------------------------------------------------
void UpsHid::hexlog_line_(const char *prefix, const uint8_t *p, int n) {
  // imprime hasta 32 bytes por línea
  const int CHUNK = 32;
  for (int i = 0; i < n; i += CHUNK) {
    int k = (n - i) < CHUNK ? (n - i) : CHUNK;
    char buf[3 * CHUNK + 1];
    int w = 0;
    for (int j = 0; j < k; j++) {
      w += snprintf(buf + w, sizeof(buf) - w, "%02X%s", p[i + j], (j + 1 < k ? " " : ""));
      if (w >= (int) sizeof(buf)) break;
    }
    ESP_LOGI(TAG, "%s%s", prefix, buf);
  }
}

// -----------------------------------------------------
// GET DESCRIPTOR: DEVICE (18 bytes)
// -----------------------------------------------------
bool UpsHid::read_device_descriptor_(usb_host_client_handle_t client,
                                     usb_device_handle_t dev_handle,
                                     uint16_t &vid, uint16_t &pid) {
  vid = pid = 0;
  if (!client || !dev_handle) return false;

  const int want = 18;
  const int total = USB_SETUP_PACKET_SIZE + want;

  usb_transfer_t *x = nullptr;
  if (usb_host_transfer_alloc(total, 0, &x) != ESP_OK) {
    ESP_LOGW(TAG, "[devdesc] alloc failed");
    return false;
  }

  usb_setup_packet_t *s = (usb_setup_packet_t *) x->data_buffer;
  s->bmRequestType = 0x80;             // D2H, std, device
  s->bRequest      = 0x06;             // GET_DESCRIPTOR
  s->wValue        = (uint16_t)((1 << 8) | 0);  // DEVICE desc, idx 0
  s->wIndex        = 0;
  s->wLength       = want;

  x->num_bytes = total;
  x->callback = ctrl_transfer_cb_;
  x->context = nullptr;
  x->device_handle = dev_handle;
  x->bEndpointAddress = 0x00;
  x->flags = 0;

  if (usb_host_transfer_submit_control(client, x) != ESP_OK) {
    ESP_LOGW(TAG, "[devdesc] submit failed");
    usb_host_transfer_free(x);
    return false;
  }

  // Espera activa despachando eventos del cliente hasta estado terminal
  TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(1500);
  while (xTaskGetTickCount() < deadline) {
    (void) usb_host_client_handle_events(client, pdMS_TO_TICKS(10));
    if (x->status == USB_TRANSFER_STATUS_COMPLETED ||
        x->status == USB_TRANSFER_STATUS_ERROR ||
        x->status == USB_TRANSFER_STATUS_STALL ||
        x->status == USB_TRANSFER_STATUS_NO_DEVICE ||
        x->status == USB_TRANSFER_STATUS_CANCELED) {
      break;
    }
  }

  if (x->status != USB_TRANSFER_STATUS_COMPLETED) {
    ESP_LOGW(TAG, "[devdesc] status=0x%X", (unsigned) x->status);
    usb_host_transfer_free(x);
    return false;
  }

  const uint8_t *d = x->data_buffer + USB_SETUP_PACKET_SIZE;
  if (d[1] != 1 || d[0] < 18) {
    ESP_LOGW(TAG, "[devdesc] invalid header (bLen=%u, bDescType=%u)", d[0], d[1]);
    usb_host_transfer_free(x);
    return false;
  }

  // idVendor (offset 8..9), idProduct (10..11)
  vid = (uint16_t)(d[8] | (d[9] << 8));
  pid = (uint16_t)(d[10] | (d[11] << 8));
  ESP_LOGI(TAG, "[devdesc] VID=0x%04X PID=0x%04X", (unsigned)vid, (unsigned)pid);

  usb_host_transfer_free(x);
  return true;
}

// -----------------------------------------------------
// GET DESCRIPTOR: CONFIG (header + full). Extrae IF HID y EP IN
// -----------------------------------------------------
bool UpsHid::read_config_descriptor_and_log_hid_(usb_host_client_handle_t client,
                                                 usb_device_handle_t dev_handle,
                                                 uint8_t &if_num, uint8_t &ep_in,
                                                 uint16_t &mps, uint8_t &interval,
                                                 uint16_t &rdesc_len) {
  if_num = 0xFF; ep_in = 0; mps = 0; interval = 0; rdesc_len = 0;
  if (!client || !dev_handle) return false;

  // A) header (9 bytes)
  const int hdr_len = 9;
  const int hdr_tot = USB_SETUP_PACKET_SIZE + hdr_len;
  usb_transfer_t *xhdr = nullptr;
  if (usb_host_transfer_alloc(hdr_tot, 0, &xhdr) != ESP_OK) {
    ESP_LOGW(TAG, "[cfg] alloc header failed");
    return false;
  }
  {
    usb_setup_packet_t *s = (usb_setup_packet_t *) xhdr->data_buffer;
    s->bmRequestType = 0x80; s->bRequest = 0x06; // GET_DESCRIPTOR
    s->wValue = (uint16_t)((2 << 8) | 0);        // CONFIG, idx 0
    s->wIndex = 0; s->wLength = hdr_len;
    xhdr->num_bytes = hdr_tot; xhdr->callback = ctrl_transfer_cb_;
    xhdr->context = nullptr; xhdr->device_handle = dev_handle;
    xhdr->bEndpointAddress = 0x00; xhdr->flags = 0;
  }
  if (usb_host_transfer_submit_control(client, xhdr) != ESP_OK) {
    ESP_LOGW(TAG, "[cfg] submit header failed");
    usb_host_transfer_free(xhdr);
    return false;
  }
  {
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(1500);
    while (xTaskGetTickCount() < deadline) {
      (void) usb_host_client_handle_events(client, pdMS_TO_TICKS(10));
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
  uint16_t wTotalLength = (uint16_t)(cfg_hdr[2] | (cfg_hdr[3] << 8));
  usb_host_transfer_free(xhdr);

  // B) config completa
  int payload = wTotalLength;
  int total   = USB_SETUP_PACKET_SIZE + payload;
  usb_transfer_t *xf = nullptr;
  if (usb_host_transfer_alloc(total, 0, &xf) != ESP_OK) {
    ESP_LOGW(TAG, "[cfg] alloc full failed");
    return false;
  }
  {
    usb_setup_packet_t *s = (usb_setup_packet_t *) xf->data_buffer;
    s->bmRequestType = 0x80; s->bRequest = 0x06;
    s->wValue = (uint16_t)((2 << 8) | 0);
    s->wIndex = 0; s->wLength = payload;
    xf->num_bytes = total; xf->callback = ctrl_transfer_cb_;
    xf->context = nullptr; xf->device_handle = dev_handle;
    xf->bEndpointAddress = 0x00; xf->flags = 0;
  }
  if (usb_host_transfer_submit_control(client, xf) != ESP_OK) {
    ESP_LOGW(TAG, "[cfg] submit full failed");
    usb_host_transfer_free(xf);
    return false;
  }
  {
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(1500);
    while (xTaskGetTickCount() < deadline) {
      (void) usb_host_client_handle_events(client, pdMS_TO_TICKS(10));
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

  // C) parse: INTERFACE HID + ENDPOINT IN + longitud Report Descriptor (de HID descriptor)
  const uint8_t *p   = xf->data_buffer + USB_SETUP_PACKET_SIZE;
  const uint8_t *end = p + payload;
  int hid_if = -1;

  while (p + 2 <= end && p[0] >= 2 && p + p[0] <= end) {
    uint8_t len = p[0], type = p[1];
    if (type == 0x04 && len >= 9) {  // INTERFACE
      uint8_t ifnum = p[2];
      uint8_t bClass = p[5], bSub = p[6], bProto = p[7];
      if (bClass == 0x03 && hid_if < 0) {
        hid_if = ifnum;
        ESP_LOGI(TAG, "[cfg] HID IF=%d class=0x%02X sub=0x%02X proto=0x%02X",
                 (int) ifnum, bClass, bSub, bProto);
      }
    } else if (type == 0x21 && len >= 9 && hid_if >= 0 && if_num == 0xFF) {  // HID descriptor
      // HID descriptor tiene wDescriptorLength en offset 7..8
      rdesc_len = (uint16_t)(p[7] | (p[8] << 8));
    } else if (type == 0x05 && len >= 7 && hid_if >= 0 && ep_in == 0) {  // ENDPOINT
      uint8_t bEndpointAddress = p[2];
      bool is_in   = (bEndpointAddress & 0x80) != 0;
      bool is_intr = ((p[3] & 0x03) == 3);
      if (is_in && is_intr) {
        ep_in   = bEndpointAddress;
        mps     = (uint16_t)(p[4] | (p[5] << 8));
        interval= p[6];
        if_num  = (uint8_t) hid_if;
      }
    }
    p += len;
  }

  usb_host_transfer_free(xf);

  if (if_num != 0xFF && ep_in != 0) {
    ESP_LOGI(TAG, "[cfg] HID endpoint IN=0x%02X MPS=%u interval=%u ms",
             ep_in, (unsigned) mps, (unsigned) interval);
    if (rdesc_len > 0)
      ESP_LOGI(TAG, "[cfg] Report Descriptor length=%u bytes", (unsigned) rdesc_len);
    else
      ESP_LOGW(TAG, "[cfg] HID descriptor sin wDescriptorLength (usaremos chunked GET_DESCRIPTOR)");
    return true;
  } else {
    ESP_LOGW(TAG, "[cfg] No se encontró interfaz HID o endpoint IN.");
    return false;
  }
}

// -----------------------------------------------------
// HID GET_DESCRIPTOR (Report) en trozos y log
// -----------------------------------------------------
bool UpsHid::dump_report_descriptor_chunked_(usb_host_client_handle_t client,
                                             usb_device_handle_t dev_handle,
                                             uint8_t if_num,
                                             uint16_t &out_total_len) {
  out_total_len = 0;
  if (!client || !dev_handle) return false;

  // Pedimos por bloques de 128 bytes hasta que falle o no devuelva datos
  const int CHUNK = 128;
  for (int offs = 0; offs < 1024; offs += CHUNK) {
    int want = CHUNK;
    int total = USB_SETUP_PACKET_SIZE + want;
    usb_transfer_t *x = nullptr;
    if (usb_host_transfer_alloc(total, 0, &x) != ESP_OK) {
      ESP_LOGW(TAG, "[rdesc] alloc failed");
      return (offs > 0);
    }

    usb_setup_packet_t *s = (usb_setup_packet_t *) x->data_buffer;
    s->bmRequestType = 0x81; // D2H, std, interface
    s->bRequest      = 0x06; // GET_DESCRIPTOR
    // HID Report descriptor type = 0x22
    s->wValue        = (uint16_t)((0x22 << 8) | 0);
    s->wIndex        = if_num;
    s->wLength       = want;

    x->num_bytes = total;
    x->callback = ctrl_transfer_cb_;
    x->context = nullptr;
    x->device_handle = dev_handle;
    x->bEndpointAddress = 0x00;
    x->flags = 0;

    if (usb_host_transfer_submit_control(client, x) != ESP_OK) {
      usb_host_transfer_free(x);
      break;
    }

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(1500);
    while (xTaskGetTickCount() < deadline) {
      (void) usb_host_client_handle_events(client, pdMS_TO_TICKS(10));
      if (x->status == USB_TRANSFER_STATUS_COMPLETED ||
          x->status == USB_TRANSFER_STATUS_ERROR ||
          x->status == USB_TRANSFER_STATUS_STALL ||
          x->status == USB_TRANSFER_STATUS_NO_DEVICE ||
          x->status == USB_TRANSFER_STATUS_CANCELED) {
        break;
      }
    }

    if (x->status != USB_TRANSFER_STATUS_COMPLETED) {
      usb_host_transfer_free(x);
      break;
    }

    const uint8_t *d = x->data_buffer + USB_SETUP_PACKET_SIZE;
    int got = x->actual_num_bytes;
    if (got <= 0) {
      usb_host_transfer_free(x);
      break;
    }
    if (offs == 0) {
      ESP_LOGI(TAG, "[rdesc] len (chunked) ~>= %d bytes", got);
    }
    out_total_len += got;

    // Log en líneas (evita bloqueos largos)
    hexlog_line_("[rdesc] ", d, got);

    usb_host_transfer_free(x);
    if (got < CHUNK) break;  // último trozo
  }

  return (out_total_len > 0);
}

// -----------------------------------------------------
// HID: GET_REPORT (Input) por control
// -----------------------------------------------------
bool UpsHid::hid_get_report_input_ctrl_(usb_host_client_handle_t client,
                                        usb_device_handle_t dev_handle,
                                        uint8_t report_id,
                                        uint8_t *out_buf, int out_cap,
                                        int &out_len,
                                        uint8_t if_num) {
  out_len = 0;
  if (!client || !dev_handle || !out_buf || out_cap <= 0) return false;

  int want = out_cap;
  int total = USB_SETUP_PACKET_SIZE + want;
  usb_transfer_t *x = nullptr;

  if (usb_host_transfer_alloc(total, 0, &x) != ESP_OK) {
    ESP_LOGW(TAG, "[poll] alloc failed");
    return false;
  }

  // GET_REPORT (Input=1) por control:
  // bmRequestType = 0xA1 (D2H, Class, Interface)
  // bRequest      = 0x01 (GET_REPORT)
  // wValue        = (report_type<<8) | report_id  -> report_type=1 (Input)
  // wIndex        = interface
  // wLength       = buffer length
  usb_setup_packet_t *s = (usb_setup_packet_t *) x->data_buffer;
  s->bmRequestType = 0xA1;
  s->bRequest      = 0x01;
  s->wValue        = (uint16_t)((0x01 << 8) | report_id);
  s->wIndex        = if_num;
  s->wLength       = (uint16_t) want;

  x->num_bytes = total;
  x->callback  = ctrl_transfer_cb_;
  x->context   = nullptr;
  x->device_handle = dev_handle;
  x->bEndpointAddress = 0x00;
  x->flags = 0;

  if (usb_host_transfer_submit_control(client, x) != ESP_OK) {
    ESP_LOGW(TAG, "[poll] submit failed");
    usb_host_transfer_free(x);
    return false;
  }

  TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(1000);
  while (xTaskGetTickCount() < deadline) {
    (void) usb_host_client_handle_events(client, pdMS_TO_TICKS(10));
    if (x->status == USB_TRANSFER_STATUS_COMPLETED ||
        x->status == USB_TRANSFER_STATUS_ERROR ||
        x->status == USB_TRANSFER_STATUS_STALL ||
        x->status == USB_TRANSFER_STATUS_NO_DEVICE ||
        x->status == USB_TRANSFER_STATUS_CANCELED) {
      break;
    }
  }

  bool ok = false;
  if (x->status == USB_TRANSFER_STATUS_COMPLETED) {
    int got = x->actual_num_bytes;
    if (got > 0) {
      if (got > out_cap) got = out_cap;
      memcpy(out_buf, x->data_buffer + USB_SETUP_PACKET_SIZE, got);
      out_len = got;
      ok = true;
    }
  } else {
    ESP_LOGW(TAG, "[poll] status=0x%X", (unsigned) x->status);
  }

  usb_host_transfer_free(x);
  return ok;
}

// -----------------------------------------------------
// (Opcional) Decodificador mínimo – por ahora sólo stub
// -----------------------------------------------------
void UpsHid::parse_report_minimal_(uint8_t report_id, const uint8_t *d, int n) {
#ifdef UPSHID_PARSE_MINIMAL
  // Aquí añadiremos mapeos (AC presente, batería %, etc.) según NUT/hidreport
  (void) report_id; (void) d; (void) n;
#endif
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
    // 4) Tarea que despacha eventos + polling soft
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
  (void) arg;
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

  // Acumulador para generar ~1s con ciclos de 100 ms
  int tick10 = 0;
  uint8_t tmp[64];
  int out_len = 0;

  while (true) {
    esp_err_t err = usb_host_client_handle_events(self->client_, pdMS_TO_TICKS(100));
    if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
      ESP_LOGW(TAG, "[usbh_client] handle_events err=0x%X", (unsigned) err);
    }

    // 1) Descubrimiento (fuera del callback)
    if (g_probe_pending && self->dev_handle_ != nullptr) {
      // 1.a) Filtro VID/PID
      uint16_t vid = 0, pid = 0;
      if (read_device_descriptor_(self->client_, self->dev_handle_, vid, pid)) {
        if (!(vid == 0x0463 && pid == 0x0001)) {
          ESP_LOGW(TAG, "[attach] no-MGE device (VID=0x%04X PID=0x%04X) -> close", vid, pid);
          usb_host_device_close(self->client_, self->dev_handle_);
          self->dev_handle_ = nullptr;
          self->dev_addr_   = 0;
          self->hid_if_     = 0xFF;
          self->hid_ep_in_  = 0;
          g_probe_pending = false;
          continue;
        }
      } else {
        // si no podemos leer device desc, cerramos por seguridad
        ESP_LOGW(TAG, "[attach] cannot read device descriptor -> close");
        usb_host_device_close(self->client_, self->dev_handle_);
        self->dev_handle_ = nullptr;
        self->dev_addr_   = 0;
        self->hid_if_     = 0xFF;
        self->hid_ep_in_  = 0;
        g_probe_pending = false;
        continue;
      }

      uint8_t ifn, ep; uint16_t mps; uint8_t itv; uint16_t rdlen;
      if (read_config_descriptor_and_log_hid_(self->client_, self->dev_handle_,
                                              ifn, ep, mps, itv, rdlen)) {
        self->hid_if_         = ifn;
        self->hid_ep_in_      = ep;
        self->hid_ep_mps_     = mps;
        self->hid_ep_interval_= itv;
        ESP_LOGI(TAG, "[cfg] ready: IF=%u EP=0x%02X MPS=%u interval=%u",
                 (unsigned) self->hid_if_, (unsigned) self->hid_ep_in_,
                 (unsigned) self->hid_ep_mps_, (unsigned) self->hid_ep_interval_);

        // Report Descriptor (chunked)
        uint16_t tot=0;
        if (dump_report_descriptor_chunked_(self->client_, self->dev_handle_, self->hid_if_, tot)) {
          ESP_LOGI(TAG, "[rdesc] len=%u bytes", (unsigned) tot);
        }
      }
      g_probe_pending = false;
    }

    // 2) Polling cada ~1 s (si tenemos interfaz HID válida)
    if (self->dev_handle_ != nullptr && self->hid_if_ != 0xFF) {
      tick10++;
      if (tick10 >= 10) { // ~10*100ms
        tick10 = 0;

        // Secuencia 0x01, 0x64, 0x66 (como en tus trazas)
        const uint8_t ids[3] = {0x01, 0x64, 0x66};
        for (uint8_t id : ids) {
          memset(tmp, 0, sizeof(tmp));
          out_len = 0;
          if (hid_get_report_input_ctrl_(self->client_, self->dev_handle_, id,
                                         tmp, sizeof(tmp), out_len, self->hid_if_)) {
            // Log corto
            // Muestra primeros 64 bytes o los que lleguen
            int show = out_len;
            if (show > 64) show = 64;
            char pref[64];
            snprintf(pref, sizeof(pref), "[poll] GET_REPORT id=0x%02X len=%d data=", id, out_len);
            hexlog_line_(pref, tmp, show);
            // Lanza parser mínimo (stub)
            parse_report_minimal_(id, tmp, out_len);
          }
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
        // lanzar descubrimiento fuera del callback
        g_probe_pending = true;
      } else {
        ESP_LOGW(TAG, "[attach] NEW_DEV addr=%u but open failed: 0x%X",
                 (unsigned) msg->new_dev.address, (unsigned) e);
      }
      break;
    }
    case USB_HOST_CLIENT_EVENT_DEV_GONE: {
      if (self->dev_handle_ != nullptr) {
        usb_host_device_close(self->client_, self->dev_handle_);
        self->dev_handle_ = nullptr;
      }
      self->dev_addr_ = 0;
      self->hid_if_ = 0xFF;
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

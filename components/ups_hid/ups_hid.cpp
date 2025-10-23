#include "ups_hid.h"
#include "usb/usb_types_ch9.h"  // usb_setup_packet_t, USB_SETUP_PACKET_SIZE

namespace esphome {
namespace ups_hid {

static const char *const TAG = "ups_hid";

// Guardamos un puntero global para callbacks C
static UpsHid *g_self = nullptr;

// Callback “no-op” requerido por IDF en transfers de control
static void ctrl_transfer_cb_(usb_transfer_t *x) { (void) x; }

// -----------------------------------------------------
// Utilidad: loguear una línea de hex sin saturar el logger
// -----------------------------------------------------
void UpsHid::log_hex_line_(const char *tag, const uint8_t *data, int len) {
  // Máx ~48 bytes por línea para no bloquear
  const int max_line = 48;
  char buf[max_line * 3 + 1];
  int o = 0;
  for (int i = 0; i < len; i++) {
    if (o > (int) sizeof(buf) - 4) break;
    o += snprintf(buf + o, sizeof(buf) - o, "%02X%s", data[i], (i + 1 < len ? " " : ""));
  }
  buf[o] = 0;
  ESP_LOGI(TAG, "%s%s", tag, buf);
}

// -----------------------------------------------------
// GET_DESCRIPTOR(Configuration) -> parsea HID IF/EP y devuelve wTotalLength & rdesc len aproximada
// -----------------------------------------------------
bool UpsHid::read_config_descriptor_and_log_hid_(usb_host_client_handle_t client,
                                                  usb_device_handle_t dev_handle,
                                                  uint8_t &if_num, uint8_t &ep_in,
                                                  uint16_t &mps, uint8_t &interval,
                                                  uint16_t &rdesc_len) {
  if_num = 0xFF; ep_in = 0; mps = 0; interval = 0; rdesc_len = 0;
  if (!client || !dev_handle) return false;

  // 1) Leer cabecera de Configuration (9 bytes)
  const int hdr_len = 9;
  const int hdr_tot = USB_SETUP_PACKET_SIZE + hdr_len;

  usb_transfer_t *xhdr = nullptr;
  if (usb_host_transfer_alloc(hdr_tot, 0, &xhdr) != ESP_OK) {
    ESP_LOGW(TAG, "[cfg] transfer_alloc header failed");
    return false;
  }

  usb_setup_packet_t *sh = (usb_setup_packet_t *) xhdr->data_buffer;
  sh->bmRequestType = 0x80;         // IN | Standard | Device
  sh->bRequest      = 0x06;         // GET_DESCRIPTOR
  sh->wValue        = (uint16_t) ((2 << 8) | 0);  // CONFIGURATION (2), index 0
  sh->wIndex        = 0;
  sh->wLength       = hdr_len;

  xhdr->num_bytes        = hdr_tot;
  xhdr->callback         = ctrl_transfer_cb_;
  xhdr->context          = nullptr;
  xhdr->device_handle    = dev_handle;
  xhdr->bEndpointAddress = 0x00;
  xhdr->flags            = 0;

  if (usb_host_transfer_submit_control(client, xhdr) != ESP_OK) {
    ESP_LOGW(TAG, "[cfg] submit header failed");
    usb_host_transfer_free(xhdr);
    return false;
  }

  // Esperar fin (sin usar NOT_READY)
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
  uint16_t wTotalLength = (uint16_t) (cfg_hdr[2] | (cfg_hdr[3] << 8));
  usb_host_transfer_free(xhdr);

  // 2) Leer descriptor completo
  const int payload = wTotalLength;
  const int total   = USB_SETUP_PACKET_SIZE + payload;

  usb_transfer_t *xf = nullptr;
  if (usb_host_transfer_alloc(total, 0, &xf) != ESP_OK) {
    ESP_LOGW(TAG, "[cfg] transfer_alloc full failed");
    return false;
  }

  usb_setup_packet_t *sf = (usb_setup_packet_t *) xf->data_buffer;
  sf->bmRequestType = 0x80; sf->bRequest = 0x06; sf->wValue = (uint16_t) ((2 << 8) | 0);
  sf->wIndex        = 0;    sf->wLength  = payload;

  xf->num_bytes        = total;
  xf->callback         = ctrl_transfer_cb_;
  xf->context          = nullptr;
  xf->device_handle    = dev_handle;
  xf->bEndpointAddress = 0x00;
  xf->flags            = 0;

  if (usb_host_transfer_submit_control(client, xf) != ESP_OK) {
    ESP_LOGW(TAG, "[cfg] submit full failed");
    usb_host_transfer_free(xf);
    return false;
  }

  {
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(2000);
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

  // 3) Parsear para localizar INTERFACE HID y ENDPOINT IN interrupt, y extraer la HID descriptor len
  const uint8_t *p   = xf->data_buffer + USB_SETUP_PACKET_SIZE;
  const uint8_t *end = p + payload;
  int hid_if = -1;

  while (p + 2 <= end && p[0] >= 2 && p + p[0] <= end) {
    uint8_t len = p[0];
    uint8_t type = p[1];

    if (type == 4 && len >= 9) {  // INTERFACE
      uint8_t bInterfaceNumber = p[2];
      uint8_t bClass  = p[5], bSub = p[6], bProto = p[7];
      if (bClass == 0x03 && hid_if < 0) {
        hid_if = (int) bInterfaceNumber;
        ESP_LOGI(TAG, "[cfg] HID IF=%d class=0x%02X sub=0x%02X proto=0x%02X",
                 (int) bInterfaceNumber, bClass, bSub, bProto);
      }
    } else if (type == 0x21 && len >= 6 && hid_if >= 0) { // HID descriptor
      // wDescriptorLength para Report (bDescriptorType=0x22) puede venir tras el/los items
      // Estructura mínima: bLength bDescriptorType bcdHID(2) bCountry bNumDesc
      // Luego: bDescType(=0x22) wDescriptorLength(2) [repetible]
      if (len >= 9 && p[1] == 0x21) {
        uint8_t bNumDesc = p[5];
        const uint8_t *q = p + 6;
        for (uint8_t i = 0; i < bNumDesc && q + 3 <= p + len; i++, q += 3) {
          uint8_t bDescType = q[0];
          uint16_t wLen = (uint16_t) (q[1] | (q[2] << 8));
          if (bDescType == 0x22) rdesc_len = wLen;
        }
      }
    } else if (type == 5 && len >= 7 && hid_if >= 0 && ep_in == 0) { // ENDPOINT
      uint8_t bEndpointAddress = p[2];
      bool is_in   = (bEndpointAddress & 0x80) != 0;
      bool is_intr = ((p[3] & 0x03) == 3);
      if (is_in && is_intr) {
        ep_in   = bEndpointAddress;
        mps     = (uint16_t) (p[4] | (p[5] << 8));
        interval = p[6];
      }
    }

    p += len;
  }

  bool ok = (hid_if >= 0) && (ep_in != 0);
  if (ok) {
    if_num = (uint8_t) hid_if;
    ESP_LOGI(TAG, "[cfg] HID endpoint IN=0x%02X MPS=%u interval=%u ms",
             ep_in, (unsigned) mps, (unsigned) interval);
  } else {
    ESP_LOGW(TAG, "[cfg] No HID IF/EP found.");
  }

  usb_host_transfer_free(xf);
  return ok;
}

// -----------------------------------------------------
// GET_DESCRIPTOR(Report) y log troceado
// -----------------------------------------------------
bool UpsHid::dump_report_descriptor_chunked_(usb_host_client_handle_t client,
                                             usb_device_handle_t dev_handle,
                                             uint8_t if_num,
                                             uint16_t &out_len) {
  out_len = 0;
  if (!client || !dev_handle) return false;

  // Si conocemos longitud esperada, úsala; si no, pide 512 (la UPS te dio ~604 una vez).
  uint16_t want = 512;

  const int total = USB_SETUP_PACKET_SIZE + want;
  usb_transfer_t *x = nullptr;
  if (usb_host_transfer_alloc(total, 0, &x) != ESP_OK) {
    ESP_LOGW(TAG, "[rdesc] transfer_alloc failed");
    return false;
  }

  usb_setup_packet_t *s = (usb_setup_packet_t *) x->data_buffer;
  s->bmRequestType = 0x81;                 // IN | Standard | Interface
  s->bRequest      = 0x06;                 // GET_DESCRIPTOR
  s->wValue        = (uint16_t) ((0x22 << 8) | 0);   // REPORT(0x22), index 0
  s->wIndex        = if_num;
  s->wLength       = want;

  x->num_bytes        = total;
  x->callback         = ctrl_transfer_cb_;
  x->context          = nullptr;
  x->device_handle    = dev_handle;
  x->bEndpointAddress = 0x00;
  x->flags            = 0;

  if (usb_host_transfer_submit_control(client, x) != ESP_OK) {
    ESP_LOGW(TAG, "[rdesc] submit failed");
    usb_host_transfer_free(x);
    return false;
  }

  {
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(2500);
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
  }

  if (x->status != USB_TRANSFER_STATUS_COMPLETED) {
    ESP_LOGW(TAG, "[rdesc] status=0x%X", (unsigned) x->status);
    usb_host_transfer_free(x);
    return false;
  }

  int got = x->actual_num_bytes;
  if (got < USB_SETUP_PACKET_SIZE) got = USB_SETUP_PACKET_SIZE;
  got -= USB_SETUP_PACKET_SIZE;
  if (got < 0) got = 0;

  const uint8_t *rd = x->data_buffer + USB_SETUP_PACKET_SIZE;
  out_len = (uint16_t) got;

  ESP_LOGI(TAG, "[rdesc] len=%d bytes", (int) got);

  // Log en bloques de 32 bytes (rápido y no bloquea tanto)
  for (int off = 0; off < got; off += 32) {
    int n = (off + 32 <= got) ? 32 : (got - off);
    log_hex_line_("[rdesc] ", rd + off, n);
    // ceder CPU si el descriptor es largo
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  usb_host_transfer_free(x);
  return true;
}

// -----------------------------------------------------
// HID GET_REPORT (Input) por control transfer
// -----------------------------------------------------
bool UpsHid::hid_get_report_input_ctrl_(usb_host_client_handle_t client,
                                        usb_device_handle_t dev_handle,
                                        uint8_t if_num,
                                        uint8_t report_id,
                                        uint8_t *out_buf, int out_buf_sz, int &out_len,
                                        uint8_t report_type) {
  out_len = 0;
  if (!client || !dev_handle || !out_buf || out_buf_sz <= 0) return false;

  const int total = USB_SETUP_PACKET_SIZE + out_buf_sz;
  usb_transfer_t *x = nullptr;
  if (usb_host_transfer_alloc(total, 0, &x) != ESP_OK) {
    ESP_LOGW(TAG, "[poll] xfer alloc failed");
    return false;
  }

  usb_setup_packet_t *setup = (usb_setup_packet_t *) x->data_buffer;
  setup->bmRequestType = 0xA1;                        // IN | Class | Interface
  setup->bRequest      = 0x01;                        // GET_REPORT
  setup->wValue        = (uint16_t) ((report_type << 8) | report_id); // type<<8 | id
  setup->wIndex        = if_num;
  setup->wLength       = (uint16_t) out_buf_sz;

  x->num_bytes        = total;
  x->callback         = ctrl_transfer_cb_;
  x->context          = nullptr;
  x->device_handle    = dev_handle;
  x->bEndpointAddress = 0x00;
  x->flags            = 0;

  if (usb_host_transfer_submit_control(client, x) != ESP_OK) {
    ESP_LOGW(TAG, "[poll] submit failed");
    usb_host_transfer_free(x);
    return false;
  }

  {
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(1200);
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
  }

  if (x->status != USB_TRANSFER_STATUS_COMPLETED) {
    ESP_LOGW(TAG, "[poll] status=0x%X", (unsigned) x->status);
    usb_host_transfer_free(x);
    return false;
  }

  int got = x->actual_num_bytes - USB_SETUP_PACKET_SIZE;
  if (got < 0) got = 0;
  if (got > out_buf_sz) got = out_buf_sz;
  std::memcpy(out_buf, x->data_buffer + USB_SETUP_PACKET_SIZE, (size_t) got);
  out_len = got;

  usb_host_transfer_free(x);
  return true;
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

  // 2) Tarea daemon
  xTaskCreatePinnedToCore(UpsHid::host_daemon_task_, "usbh_daemon",
                          4096, nullptr, 5, nullptr, tskNO_AFFINITY);

  // 3) Registrar cliente asíncrono con callback
  usb_host_client_config_t client_cfg = {
      .is_synchronous    = false,
      .max_num_event_msg = 8,
      .async = {
          .client_event_callback = UpsHid::client_callback_,
          .callback_arg          = this,
      },
  };
  err = usb_host_client_register(&client_cfg, &this->client_);
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "USB Host client registered.");
    xTaskCreatePinnedToCore(UpsHid::client_task_, "usbh_client",
                            4096, this, 5, nullptr, tskNO_AFFINITY);
  } else {
    ESP_LOGE(TAG, "usb_host_client_register() failed: 0x%X", (unsigned) err);
    return;
  }

  g_self = this;

  // 4) Sondeo periódico mediante control GET_REPORT (1 Hz)
  this->set_interval("hid_poll", 1000, [this]() {
    if (!this->client_ || !this->dev_handle_ || this->hid_if_ == 0xFF) return;

    // Secuencia de IDs: 0x01, 0x64, 0x66 (rotando)
    static const uint8_t ids[] = {0x01, 0x64, 0x66};
    uint8_t id = ids[this->poll_idx_ % (sizeof(ids))];
    this->poll_idx_++;

    uint8_t buf[64];
    int got = 0;
    if (hid_get_report_input_ctrl_(this->client_, this->dev_handle_, this->hid_if_, id,
                                   buf, sizeof(buf), got, 0x01 /* INPUT */)) {
      // Log resumen: primeras ~48 bytes
      char tag[64];
      snprintf(tag, sizeof(tag), "[poll] GET_REPORT id=0x%02X len=%d data=", id, got);
      int show = (got > 64) ? 64 : got;
      log_hex_line_(tag, buf, show);
    }
  });
}

void UpsHid::dump_config() {
  ESP_LOGCONFIG(TAG, "UPS HID component ready.");
}

// ---------- Tareas / callbacks ----------

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

  while (true) {
    esp_err_t err = usb_host_client_handle_events(self->client_, pdMS_TO_TICKS(100));
    if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
      ESP_LOGW(TAG, "[usbh_client] handle_events err=0x%X", (unsigned) err);
    }

    // ¿Hay que sondear/descubrir HID? (lanzado fuera del callback)
    if (self->probe_pending_ && self->dev_handle_ != nullptr) {
      uint8_t ifn = 0xFF, ep = 0; uint16_t mps = 0; uint8_t itv = 0; uint16_t rdlen = 0;
      if (read_config_descriptor_and_log_hid_(self->client_, self->dev_handle_,
                                              ifn, ep, mps, itv, rdlen)) {
        self->hid_if_          = ifn;
        self->hid_ep_in_       = ep;
        self->hid_ep_mps_      = mps;
        self->hid_ep_interval_ = itv;
        self->hid_rdesc_len_   = rdlen;

        ESP_LOGI(TAG, "[cfg] ready: IF=%u EP=0x%02X MPS=%u interval=%u",
                 (unsigned) self->hid_if_, (unsigned) self->hid_ep_in_,
                 (unsigned) self->hid_ep_mps_, (unsigned) self->hid_ep_interval_);

        // Leer y volcar Report Descriptor (troceado)
        uint16_t got_len = 0;
        (void) dump_report_descriptor_chunked_(self->client_, self->dev_handle_, self->hid_if_, got_len);
      }
      self->probe_pending_ = false;
    }
  }
}

void UpsHid::client_callback_(const usb_host_client_event_msg_t *msg, void *arg) {
  auto *self = static_cast<UpsHid *>(arg);
  if (!self) return;

  switch (msg->event) {
    case USB_HOST_CLIENT_EVENT_NEW_DEV: {
      esp_err_t e = usb_host_device_open(self->client_, msg->new_dev.address, &self->dev_handle_);
      if (e == ESP_OK) {
        self->dev_addr_ = msg->new_dev.address;
        ESP_LOGI(TAG, "[attach] NEW_DEV addr=%u (opened)", (unsigned) self->dev_addr_);
        self->probe_pending_ = true;  // hacer parse & rdesc fuera del callback
      } else {
        ESP_LOGW(TAG, "[attach] NEW_DEV addr=%u but open failed: 0x%X",
                 (unsigned) msg->new_dev.address, (unsigned) e);
      }
      break;
    }

    case USB_HOST_CLIENT_EVENT_DEV_GONE: {
      // Reset de estado
      if (self->dev_handle_ != nullptr) {
        usb_host_device_close(self->client_, self->dev_handle_);
        self->dev_handle_ = nullptr;
      }
      self->dev_addr_         = 0;
      self->hid_if_           = 0xFF;
      self->hid_ep_in_        = 0;
      self->hid_ep_mps_       = 0;
      self->hid_ep_interval_  = 0;
      self->hid_rdesc_len_    = 0;
      self->probe_pending_    = false;

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

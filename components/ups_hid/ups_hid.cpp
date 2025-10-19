#include "ups_hid.h"
#include "usb/usb_types_ch9.h"  // usb_setup_packet_t, USB_SETUP_PACKET_SIZE
#include <cstdint>

// NOTA: Este archivo está pensado para ESP-IDF 5.4.x (API de USB Host actual de ESPHome-IDF)

namespace esphome {
namespace ups_hid {

static const char *const TAG = "ups_hid";

// ----------------------------------------------
// Estado simple + banderas para trabajo fuera del callback
// ----------------------------------------------
static UpsHid *g_self = nullptr;
static volatile bool g_probe_pending = false;  // lanzar descubrimiento fuera del callback

// callback no-op para transfers de control (IDF 5.4 requiere puntero no nulo)
static void ctrl_transfer_cb_(usb_transfer_t *transfer) { (void) transfer; }

// -----------------------------------------------------
// Helper: lee Configuration Descriptor, localiza HID IF,
// endpoint IN interrupt y longitud del Report Descriptor
// -----------------------------------------------------
static bool read_config_descriptor_and_log_hid_(usb_host_client_handle_t client,
                                                usb_device_handle_t dev_handle,
                                                uint8_t &if_num, uint8_t &ep_in,
                                                uint16_t &mps, uint8_t &interval,
                                                uint16_t &rdesc_len) {
  if_num = 0xFF;
  ep_in = 0; mps = 0; interval = 0; rdesc_len = 0;
  if (!client || !dev_handle) return false;

  // A) Leer cabecera de Config (9 bytes)
  const int hdr_len = 9;
  const int hdr_tot = USB_SETUP_PACKET_SIZE + hdr_len;
  usb_transfer_t *xhdr = nullptr;
  if (usb_host_transfer_alloc(hdr_tot, 0, &xhdr) != ESP_OK) {
    ESP_LOGW(TAG, "[cfg] transfer_alloc header failed");
    return false;
  }
  {
    usb_setup_packet_t *sh = (usb_setup_packet_t *) xhdr->data_buffer;
    sh->bmRequestType = 0x80; // IN | Standard | Device
    sh->bRequest      = 0x06; // GET_DESCRIPTOR
    sh->wValue        = (uint16_t)((2 /*CONFIG*/ << 8) | 0);
    sh->wIndex        = 0;
    sh->wLength       = hdr_len;

    xhdr->num_bytes        = hdr_tot;
    xhdr->callback         = ctrl_transfer_cb_;
    xhdr->context          = nullptr;
    xhdr->device_handle    = dev_handle;
    xhdr->bEndpointAddress = 0x00; // EP0
    xhdr->flags            = 0;

    if (usb_host_transfer_submit_control(client, xhdr) != ESP_OK) {
      ESP_LOGW(TAG, "[cfg] submit header failed");
      usb_host_transfer_free(xhdr);
      return false;
    }
    TickType_t dl = xTaskGetTickCount() + pdMS_TO_TICKS(1500);
    while (xTaskGetTickCount() < dl) {
      (void) usb_host_client_handle_events(client, pdMS_TO_TICKS(10));
      if (xhdr->status == USB_TRANSFER_STATUS_COMPLETED ||
          xhdr->status == USB_TRANSFER_STATUS_ERROR ||
          xhdr->status == USB_TRANSFER_STATUS_STALL ||
          xhdr->status == USB_TRANSFER_STATUS_NO_DEVICE ||
          xhdr->status == USB_TRANSFER_STATUS_CANCELED) {
        break;
      }
    }
    if (xhdr->status != USB_TRANSFER_STATUS_COMPLETED) {
      ESP_LOGW(TAG, "[cfg] header status=0x%X", (unsigned) xhdr->status);
      usb_host_transfer_free(xhdr);
      return false;
    }
  }

  const uint8_t *cfg_hdr = xhdr->data_buffer + USB_SETUP_PACKET_SIZE;
  if (cfg_hdr[1] != 2 || cfg_hdr[0] < 9) {
    ESP_LOGW(TAG, "[cfg] invalid header bType=%u bLen=%u", cfg_hdr[1], cfg_hdr[0]);
    usb_host_transfer_free(xhdr);
    return false;
  }
  uint16_t wTotalLength = (uint16_t)(cfg_hdr[2] | (cfg_hdr[3] << 8));
  usb_host_transfer_free(xhdr);

  // B) Leer descriptor de Config COMPLETO
  int payload = wTotalLength;
  if (payload < 9) {
    ESP_LOGW(TAG, "[cfg] total length too short: %u", (unsigned) payload);
    return false;
  }
  // Limitar a algo razonable
  if (payload > 1024) payload = 1024;

  int total = USB_SETUP_PACKET_SIZE + payload;
  usb_transfer_t *xfull = nullptr;
  if (usb_host_transfer_alloc(total, 0, &xfull) != ESP_OK) {
    ESP_LOGW(TAG, "[cfg] transfer_alloc full failed");
    return false;
  }
  {
    usb_setup_packet_t *sf = (usb_setup_packet_t *) xfull->data_buffer;
    sf->bmRequestType = 0x80;
    sf->bRequest      = 0x06; // GET_DESCRIPTOR
    sf->wValue        = (uint16_t)((2 /*CONFIG*/ << 8) | 0);
    sf->wIndex        = 0;
    sf->wLength       = payload;

    xfull->num_bytes        = total;
    xfull->callback         = ctrl_transfer_cb_;
    xfull->context          = nullptr;
    xfull->device_handle    = dev_handle;
    xfull->bEndpointAddress = 0x00; // EP0
    xfull->flags            = 0;

    if (usb_host_transfer_submit_control(client, xfull) != ESP_OK) {
      ESP_LOGW(TAG, "[cfg] submit full failed");
      usb_host_transfer_free(xfull);
      return false;
    }
    TickType_t dl = xTaskGetTickCount() + pdMS_TO_TICKS(1500);
    while (xTaskGetTickCount() < dl) {
      (void) usb_host_client_handle_events(client, pdMS_TO_TICKS(10));
      if (xfull->status == USB_TRANSFER_STATUS_COMPLETED ||
          xfull->status == USB_TRANSFER_STATUS_ERROR ||
          xfull->status == USB_TRANSFER_STATUS_STALL ||
          xfull->status == USB_TRANSFER_STATUS_NO_DEVICE ||
          xfull->status == USB_TRANSFER_STATUS_CANCELED) {
        break;
      }
    }
    if (xfull->status != USB_TRANSFER_STATUS_COMPLETED) {
      ESP_LOGW(TAG, "[cfg] full status=0x%X", (unsigned) xfull->status);
      usb_host_transfer_free(xfull);
      return false;
    }
  }

  // C) Parsear hasta encontrar: INTERFACE HID y su ENDPOINT IN,
  //    y además el HID descriptor (0x21) para extraer wDescriptorLength (REPORT, 0x22)
  const uint8_t *p   = xfull->data_buffer + USB_SETUP_PACKET_SIZE;
  const uint8_t *end = p + payload;

  int hid_if = -1;
  uint8_t  ep_in_loc = 0, interval_loc = 0;
  uint16_t mps_loc   = 0;

  while (p + 2 <= end && p[0] >= 2 && p + p[0] <= end) {
    uint8_t len = p[0], type = p[1];

    if (type == 4 /*INTERFACE*/ && len >= 9) {
      uint8_t bInterfaceNumber = p[2];
      uint8_t bClass           = p[5];
      uint8_t bSub             = p[6];
      uint8_t bProto           = p[7];
      if (bClass == 0x03 /*HID*/ && hid_if < 0) {
        hid_if = bInterfaceNumber;
        if_num = bInterfaceNumber;
        ESP_LOGI(TAG, "[cfg] HID IF=%d class=0x%02X sub=0x%02X proto=0x%02X",
                 (int) bInterfaceNumber, bClass, bSub, bProto);
      }
    } else if (type == 0x21 /*HID*/ && len >= 6 && hid_if >= 0 && if_num == (uint8_t)hid_if) {
      if (len >= 9) {
        uint8_t bNum = p[5];
        const uint8_t *q = p + 6;
        for (uint8_t i = 0; i < bNum && q + 3 <= p + len; i++) {
          uint8_t  cls_desc_type = q[0];
          uint16_t cls_desc_len  = (uint16_t)(q[1] | (q[2] << 8));
          if (cls_desc_type == 0x22 /*REPORT*/) {
            rdesc_len = cls_desc_len;
          }
          q += 3;
        }
      }
    } else if (type == 5 /*ENDPOINT*/ && len >= 7 && hid_if >= 0 && if_num == (uint8_t)hid_if && ep_in_loc == 0) {
      uint8_t bEndpointAddress = p[2];
      bool is_in   = (bEndpointAddress & 0x80) != 0;
      bool is_intr = ((p[3] & 0x03) == 3);
      if (is_in && is_intr) {
        ep_in_loc     = bEndpointAddress;
        mps_loc       = (uint16_t)(p[4] | (p[5] << 8));
        interval_loc  = p[6];
      }
    }
    p += len;
  }

  usb_host_transfer_free(xfull);

  if (hid_if >= 0 && ep_in_loc != 0) {
    ESP_LOGI(TAG, "[cfg] HID endpoint IN=0x%02X MPS=%u interval=%u ms",
             ep_in_loc, (unsigned) mps_loc, (unsigned) interval_loc);
    // devolver por referencia
    ep_in    = ep_in_loc;
    mps      = mps_loc;
    interval = interval_loc;
    return true;
  } else {
    ESP_LOGW(TAG, "[cfg] No se encontró interfaz HID o endpoint IN.");
    return false;
  }
}

// -----------------------------------------------------
// Helper: lee y vuelca el Report Descriptor completo
// -----------------------------------------------------
static bool dump_report_descriptor_(usb_host_client_handle_t client,
                                    usb_device_handle_t dev_handle,
                                    uint8_t interface_number,
                                    uint16_t rdesc_len) {
  if (!client || !dev_handle) return false;
  if (rdesc_len == 0) rdesc_len = 512;  // fallback típico
  if (rdesc_len > 1024) rdesc_len = 1024;

  int total = USB_SETUP_PACKET_SIZE + rdesc_len;
  usb_transfer_t *xfer = nullptr;
  if (usb_host_transfer_alloc(total, 0, &xfer) != ESP_OK) {
    ESP_LOGW(TAG, "[rdesc] transfer_alloc failed");
    return false;
  }

  usb_setup_packet_t *su = (usb_setup_packet_t *) xfer->data_buffer;
  su->bmRequestType = 0x81;                           // IN | Standard | Interface
  su->bRequest      = 0x06;                           // GET_DESCRIPTOR
  su->wValue        = (uint16_t)((0x22 /*REPORT*/ << 8) | 0);
  su->wIndex        = interface_number;               // interface
  su->wLength       = rdesc_len;

  xfer->num_bytes        = total;
  xfer->callback         = ctrl_transfer_cb_;
  xfer->context          = nullptr;
  xfer->device_handle    = dev_handle;
  xfer->bEndpointAddress = 0x00; // EP0
  xfer->flags            = 0;

  esp_err_t se = usb_host_transfer_submit_control(client, xfer);
  if (se != ESP_OK) {
    ESP_LOGW(TAG, "[rdesc] submit failed: 0x%X", (unsigned) se);
    usb_host_transfer_free(xfer);
    return false;
  }

  // Esperar finalización
  {
    TickType_t dl = xTaskGetTickCount() + pdMS_TO_TICKS(2000);
    while (xTaskGetTickCount() < dl) {
      (void) usb_host_client_handle_events(client, pdMS_TO_TICKS(10));
      if (xfer->status == USB_TRANSFER_STATUS_COMPLETED ||
          xfer->status == USB_TRANSFER_STATUS_ERROR ||
          xfer->status == USB_TRANSFER_STATUS_STALL ||
          xfer->status == USB_TRANSFER_STATUS_NO_DEVICE ||
          xfer->status == USB_TRANSFER_STATUS_CANCELED) {
        break;
      }
    }
  }

  if (xfer->status != USB_TRANSFER_STATUS_COMPLETED) {
    ESP_LOGW(TAG, "[rdesc] status=0x%X", (unsigned) xfer->status);
    usb_host_transfer_free(xfer);
    return false;
  }

  // Volcado bonito en líneas de 16 bytes con pequeñas pausas
  const uint8_t *d   = xfer->data_buffer + USB_SETUP_PACKET_SIZE;
  int             n  = xfer->actual_num_bytes;
  if (n > rdesc_len) n = rdesc_len;

  ESP_LOGI(TAG, "[rdesc] len=%d bytes", n);

  for (int off = 0; off < n; off += 16) {
    int line = (n - off >= 16) ? 16 : (n - off);
    char buf[16 * 3 + 1];
    int k = 0;
    for (int i = 0; i < line; i++) {
      if (k + 3 < (int) sizeof(buf)) {
        int p2 = off + i;
        k += snprintf(buf + k, sizeof(buf) - k, "%02X%s", d[p2], (i + 1 < line ? " " : ""));
      }
    }
    buf[sizeof(buf)-1] = '\0';
    ESP_LOGI(TAG, "[rdesc] %s", buf);
    vTaskDelay(pdMS_TO_TICKS(5));  // pausita para que el logger no trunque
  }

  usb_host_transfer_free(xfer);
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
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "usb_host_client_register() failed: 0x%X", (unsigned) err);
    return;
  }

  // 4) Tarea que despacha eventos
  xTaskCreatePinnedToCore(UpsHid::client_task_, "usbh_client",
                          4096, this, 5, nullptr, tskNO_AFFINITY);

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
  while (true) {
    esp_err_t err = usb_host_client_handle_events(self->client_, pdMS_TO_TICKS(100));
    if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
      ESP_LOGW(TAG, "[usbh_client] handle_events err=0x%X", (unsigned) err);
    }

    // Descubrimiento/lectura del descriptor tras NEW_DEV (fuera del callback)
    if (g_probe_pending && self->dev_handle_ != nullptr) {
      uint8_t if_num, ep; uint16_t mps; uint8_t itv; uint16_t rdlen;
      if (read_config_descriptor_and_log_hid_(self->client_, self->dev_handle_, if_num, ep, mps, itv, rdlen)) {
        self->hid_if_num_      = if_num;
        self->hid_ep_in_       = ep;
        self->hid_ep_mps_      = mps;
        self->hid_ep_interval_ = itv;

        ESP_LOGI(TAG, "[cfg] ready: IF=%u EP=0x%02X MPS=%u interval=%u",
                 (unsigned) if_num, (unsigned) ep, (unsigned) mps, (unsigned) itv);

        // Volcado del Report Descriptor completo
        (void) dump_report_descriptor_(self->client_, self->dev_handle_, self->hid_if_num_, rdlen);
      }
      g_probe_pending = false;
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
        // Lanzar descubrimiento fuera del callback
        g_probe_pending = true;
      } else {
        ESP_LOGW(TAG, "[attach] NEW_DEV addr=%u but open failed: 0x%X",
                 (unsigned) msg->new_dev.address, (unsigned) e);
      }
      break;
    }

    case USB_HOST_CLIENT_EVENT_DEV_GONE: {
      // Cerrar y limpiar
      if (self->dev_handle_ != nullptr) {
        usb_host_device_close(self->client_, self->dev_handle_);
        self->dev_handle_ = nullptr;
      }
      self->dev_addr_        = 0;
      self->hid_if_num_      = 0xFF;
      self->hid_ep_in_       = 0;
      self->hid_ep_mps_      = 0;
      self->hid_ep_interval_ = 0;
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

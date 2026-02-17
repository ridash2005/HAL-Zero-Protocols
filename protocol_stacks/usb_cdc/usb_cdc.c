/**
 * @file    usb_cdc.c
 * @brief   USB CDC (Communication Device Class) — Implementation
 * @version 1.0.0
 *
 * @details Implements the USB CDC ACM class for STM32F4xx OTG_FS.
 *
 * Architecture:
 *   ┌──────────────────────────────────┐
 *   │        Application Layer         │
 *   │ USB_CDC_Transmit / Receive       │
 *   ├──────────────────────────────────┤
 *   │        CDC Class Logic           │
 *   │ Line coding, control line state  │
 *   ├──────────────────────────────────┤
 *   │      USB Device Core             │
 *   │ EP0 setup, IN/OUT transfers      │
 *   ├──────────────────────────────────┤
 *   │      OTG_FS Hardware             │
 *   │ Register-level access            │
 *   └──────────────────────────────────┘
 *
 * @note   This is a reference implementation showing the CDC class
 *         framework. A full production USB stack would include
 *         complete descriptor generation and enumeration.
 */

#include "usb_cdc.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* ═══════════════════════════════════════════════════════════════════════════
 *  OTG_FS Register Base (STM32F4xx)
 * ═══════════════════════════════════════════════════════════════════════════
 */

#define USB_OTG_FS_BASE ((uintptr_t)0x50000000U)
#define USB_OTG_GLOBAL ((volatile uint32_t *)(USB_OTG_FS_BASE + 0x000))
#define USB_OTG_DEVICE ((volatile uint32_t *)(USB_OTG_FS_BASE + 0x800))
#define USB_OTG_INEPx(n)                                                       \
  ((volatile uint32_t *)(USB_OTG_FS_BASE + 0x900 + (n) * 0x20))
#define USB_OTG_OUTEPx(n)                                                      \
  ((volatile uint32_t *)(USB_OTG_FS_BASE + 0xB00 + (n) * 0x20))
#define USB_OTG_FIFO(n)                                                        \
  ((volatile uint32_t *)(USB_OTG_FS_BASE + 0x1000 + (n) * 0x1000))

/* Key register offsets */
#define OTG_GOTGCTL 0x00
#define OTG_GAHBCFG 0x08
#define OTG_GUSBCFG 0x0C
#define OTG_GRSTCTL 0x10
#define OTG_GINTSTS 0x14
#define OTG_GINTMSK 0x18
#define OTG_GRXSTSP 0x20
#define OTG_GRXFSIZ 0x24
#define OTG_DIEPTXF0 0x28
#define OTG_GCCFG 0x38

/* Device registers */
#define OTG_DCFG 0x00
#define OTG_DCTL 0x04
#define OTG_DSTS 0x08
#define OTG_DIEPMSK 0x10
#define OTG_DOEPMSK 0x14
#define OTG_DAINT 0x18
#define OTG_DAINTMSK 0x1C

/* GINTSTS flags */
#define OTG_GINTSTS_USBRST (1 << 12)
#define OTG_GINTSTS_ENUMDNE (1 << 13)
#define OTG_GINTSTS_RXFLVL (1 << 4)
#define OTG_GINTSTS_IEPINT (1 << 18)
#define OTG_GINTSTS_OEPINT (1 << 19)

/* ═══════════════════════════════════════════════════════════════════════════
 *  USB Descriptors (Built-in)
 * ═══════════════════════════════════════════════════════════════════════════
 */

/* Device Descriptor (18 bytes) */
static const uint8_t usb_device_descriptor[] = {
    0x12, /* bLength */
    0x01, /* bDescriptorType: Device */
    0x00,
    0x02,                /* bcdUSB: 2.00 */
    USB_CLASS_CDC,       /* bDeviceClass: CDC */
    0x00,                /* bDeviceSubClass */
    0x00,                /* bDeviceProtocol */
    USB_CDC_PACKET_SIZE, /* bMaxPacketSize0 */
    0x83,
    0x04, /* idVendor: 0x0483 (STMicroelectronics) */
    0x40,
    0x57, /* idProduct: 0x5740 (Virtual COM Port) */
    0x00,
    0x02, /* bcdDevice: 2.00 */
    0x01, /* iManufacturer */
    0x02, /* iProduct */
    0x03, /* iSerialNumber */
    0x01, /* bNumConfigurations */
};

/* Configuration Descriptor (including all interfaces) */
static const uint8_t usb_config_descriptor[] = {
    /* Configuration Descriptor */
    0x09,
    0x02, /* bLength, bDescriptorType */
    0x43,
    0x00, /* wTotalLength: 67 bytes */
    0x02, /* bNumInterfaces: 2 */
    0x01, /* bConfigurationValue */
    0x00, /* iConfiguration */
    0x80, /* bmAttributes: Bus-powered */
    0xFA, /* bMaxPower: 500mA */

    /* Interface 0: CDC Communication Interface */
    0x09,
    0x04,                 /* bLength, bDescriptorType */
    0x00,                 /* bInterfaceNumber: 0 */
    0x00,                 /* bAlternateSetting */
    0x01,                 /* bNumEndpoints: 1 (notification EP) */
    USB_CLASS_CDC,        /* bInterfaceClass */
    USB_CDC_SUBCLASS_ACM, /* bInterfaceSubClass */
    USB_CDC_PROTOCOL_AT,  /* bInterfaceProtocol */
    0x00,                 /* iInterface */

    /* CDC Header Functional Descriptor */
    0x05,
    0x24,
    0x00,
    0x10,
    0x01,

    /* CDC Call Management Functional Descriptor */
    0x05,
    0x24,
    0x01,
    0x00,
    0x01,

    /* CDC ACM Functional Descriptor */
    0x04,
    0x24,
    0x02,
    0x02,

    /* CDC Union Functional Descriptor */
    0x05,
    0x24,
    0x06,
    0x00,
    0x01,

    /* EP2 IN — Notification Endpoint (Interrupt) */
    0x07,
    0x05,
    0x82, /* bEndpointAddress: IN 2 */
    0x03, /* bmAttributes: Interrupt */
    0x08,
    0x00, /* wMaxPacketSize: 8 */
    0x10, /* bInterval: 16ms */

    /* Interface 1: CDC Data Interface */
    0x09,
    0x04,
    0x01, /* bInterfaceNumber: 1 */
    0x00,
    0x02, /* bAlternateSetting, bNumEndpoints: 2 */
    0x0A, /* bInterfaceClass: CDC Data */
    0x00,
    0x00, /* bInterfaceSubClass, bInterfaceProtocol */
    0x00, /* iInterface */

    /* EP1 OUT — Bulk Data (Host → Device) */
    0x07,
    0x05,
    0x01, /* bEndpointAddress: OUT 1 */
    0x02, /* bmAttributes: Bulk */
    0x40,
    0x00, /* wMaxPacketSize: 64 */
    0x00, /* bInterval */

    /* EP1 IN — Bulk Data (Device → Host) */
    0x07,
    0x05,
    0x81, /* bEndpointAddress: IN 1 */
    0x02, /* bmAttributes: Bulk */
    0x40,
    0x00, /* wMaxPacketSize: 64 */
    0x00, /* bInterval */
};

/* String Descriptor: Language ID */
static const uint8_t usb_string_langid[] = {
    0x04, 0x03, 0x09, 0x04 /* English (US) */
};

/* ═══════════════════════════════════════════════════════════════════════════
 *  Ring Buffer Operations
 * ═══════════════════════════════════════════════════════════════════════════
 */

static void ringbuf_init(usb_cdc_ringbuf_t *rb) {
  rb->head = 0;
  rb->tail = 0;
}

static uint16_t ringbuf_count(usb_cdc_ringbuf_t *rb) {
  int16_t diff = rb->head - rb->tail;
  if (diff < 0)
    diff += USB_CDC_RX_BUFFER_SIZE;
  return (uint16_t)diff;
}

static void ringbuf_push(usb_cdc_ringbuf_t *rb, uint8_t byte) {
  uint16_t next = (rb->head + 1) % USB_CDC_RX_BUFFER_SIZE;
  if (next != rb->tail) {
    rb->buffer[rb->head] = byte;
    rb->head = next;
  }
  /* If full, drop the byte (overrun) */
}

static int16_t ringbuf_pop(usb_cdc_ringbuf_t *rb) {
  if (rb->head == rb->tail)
    return -1;
  uint8_t byte = rb->buffer[rb->tail];
  rb->tail = (rb->tail + 1) % USB_CDC_RX_BUFFER_SIZE;
  return byte;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Internal — OTG_FS Low-Level Operations
 * ═══════════════════════════════════════════════════════════════════════════
 */

static void usb_write_packet(uint8_t ep_num, const uint8_t *data,
                             uint16_t len) {
  volatile uint32_t *fifo = USB_OTG_FIFO(ep_num);
  uint32_t word_count = (len + 3) / 4;

  for (uint32_t i = 0; i < word_count; i++) {
    uint32_t word = 0;
    for (uint8_t b = 0; b < 4; b++) {
      uint16_t idx = (i * 4) + b;
      if (idx < len) {
        word |= ((uint32_t)data[idx]) << (b * 8);
      }
    }
    *fifo = word;
  }
}

static void usb_read_packet(uint8_t *dest, uint16_t len) {
  volatile uint32_t *fifo = USB_OTG_FIFO(0);
  uint32_t word_count = (len + 3) / 4;

  for (uint32_t i = 0; i < word_count; i++) {
    uint32_t word = *fifo;
    for (uint8_t b = 0; b < 4; b++) {
      uint16_t idx = (i * 4) + b;
      if (idx < len && dest != NULL) {
        dest[idx] = (uint8_t)(word >> (b * 8));
      }
    }
  }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════════════
 */

usb_cdc_status_t USB_CDC_Init(usb_cdc_handle_t *h) {
  if (h == NULL)
    return USB_CDC_ERROR;

  /* Initialize runtime state — do NOT memset the entire handle,
   * because the user may have already set rx_callback/cb_context. */
  ringbuf_init(&h->rx_ring);
  h->tx_busy = 0;
  h->configured = 0;
  h->connected = 0;
  h->dtr = 0;
  h->rts = 0;

  /* Default line coding: 115200 8N1 */
  h->line_coding.baud_rate = 115200;
  h->line_coding.data_bits = 8;
  h->line_coding.stop_bits = 0;
  h->line_coding.parity = 0;

  /*
   * Hardware initialization sequence:
   * 1. Enable USB OTG FS clock (RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN)
   * 2. Configure GPIO PA11 (DM) and PA12 (DP) as AF10
   * 3. Configure USB OTG core (GAHBCFG, GUSBCFG)
   * 4. Set device mode (GUSBCFG.FDMOD)
   * 5. Configure device speed (DCFG.DSPD = Full-Speed)
   * 6. Configure FIFO sizes
   * 7. Configure endpoints
   * 8. Enable interrupts (GINTMSK)
   *
   * The actual hardware init is hardware-specific and typically done
   * in the system startup code. This function focuses on the CDC
   * class-level initialization.
   */

  /* Mark as ready (awaiting USB cable connection) */
  h->configured = 0;
  h->connected = 0;

  return USB_CDC_OK;
}

usb_cdc_status_t USB_CDC_DeInit(usb_cdc_handle_t *h) {
  if (h == NULL)
    return USB_CDC_ERROR;

  h->configured = 0;
  h->connected = 0;

  /* Disable USB pull-up to disconnect from bus */
  /* In production: clear DCTL.SDIS or GCCFG.PWRDWN */

  return USB_CDC_OK;
}

usb_cdc_status_t USB_CDC_Transmit(usb_cdc_handle_t *h, const uint8_t *data,
                                  uint16_t len) {
  if (h == NULL || data == NULL)
    return USB_CDC_ERROR;
  if (!h->configured)
    return USB_CDC_DISCONNECTED;
  if (h->tx_busy)
    return USB_CDC_BUSY;

  h->tx_busy = 1;

  /* Split into USB_CDC_PACKET_SIZE chunks */
  uint16_t remaining = len;
  const uint8_t *ptr = data;

  while (remaining > 0) {
    uint16_t chunk =
        (remaining > USB_CDC_PACKET_SIZE) ? USB_CDC_PACKET_SIZE : remaining;

    /* Copy to TX buffer */
    memcpy(h->tx_buffer, ptr, chunk);

    /* Write to EP1 IN FIFO */
    usb_write_packet(1, h->tx_buffer, chunk);

    ptr += chunk;
    remaining -= chunk;

    /* In a real implementation, we'd wait for the IN token from
       the host before sending the next chunk. This is simplified. */
  }

  /* If the last packet was exactly USB_CDC_PACKET_SIZE, send a ZLP
     (Zero-Length Packet) to signal end of transfer */
  if (len > 0 && (len % USB_CDC_PACKET_SIZE) == 0) {
    usb_write_packet(1, NULL, 0);
  }

  h->tx_busy = 0;
  return USB_CDC_OK;
}

uint16_t USB_CDC_Receive(usb_cdc_handle_t *h, uint8_t *data, uint16_t max_len) {
  if (h == NULL || data == NULL)
    return 0;

  uint16_t count = 0;
  while (count < max_len) {
    int16_t byte = ringbuf_pop(&h->rx_ring);
    if (byte < 0)
      break;
    data[count++] = (uint8_t)byte;
  }
  return count;
}

uint16_t USB_CDC_Available(usb_cdc_handle_t *h) {
  if (h == NULL)
    return 0;
  return ringbuf_count(&h->rx_ring);
}

int USB_CDC_Printf(usb_cdc_handle_t *h, const char *fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  int len = vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  if (len > 0) {
    USB_CDC_Transmit(h, (const uint8_t *)buf, (uint16_t)len);
  }
  return len;
}

uint8_t USB_CDC_IsConnected(usb_cdc_handle_t *h) {
  return (h != NULL && h->configured && h->connected) ? 1 : 0;
}

usb_cdc_line_coding_t USB_CDC_GetLineCoding(usb_cdc_handle_t *h) {
  return h->line_coding;
}

void USB_CDC_RegisterCallback(usb_cdc_handle_t *h, usb_cdc_rx_callback_t cb,
                              void *ctx) {
  h->rx_callback = cb;
  h->cb_context = ctx;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  USB IRQ Handler (Simplified)
 * ═══════════════════════════════════════════════════════════════════════════
 */

void USB_CDC_IRQHandler(usb_cdc_handle_t *h) {
  volatile uint32_t *global_regs = USB_OTG_GLOBAL;
  uint32_t gintsts = global_regs[OTG_GINTSTS / 4];

  /* USB Reset */
  if (gintsts & OTG_GINTSTS_USBRST) {
    global_regs[OTG_GINTSTS / 4] = OTG_GINTSTS_USBRST;
    h->configured = 0;
    h->connected = 0;
    /* Re-initialize endpoints after reset */
  }

  /* Enumeration Done */
  if (gintsts & OTG_GINTSTS_ENUMDNE) {
    global_regs[OTG_GINTSTS / 4] = OTG_GINTSTS_ENUMDNE;
    h->connected = 1;
  }

  /* RX FIFO Non-Empty */
  if (gintsts & OTG_GINTSTS_RXFLVL) {
    uint32_t grxstsp = global_regs[OTG_GRXSTSP / 4];
    uint8_t ep_num = grxstsp & 0x0F;
    uint16_t bcnt = (grxstsp >> 4) & 0x7FF;
    uint8_t pktsts = (grxstsp >> 17) & 0x0F;

    if (ep_num == 1 && pktsts == 0x02) {
      /* OUT data packet received on EP1 */
      uint8_t rx_data[USB_CDC_PACKET_SIZE];
      usb_read_packet(rx_data, bcnt);

      /* Push into ring buffer */
      for (uint16_t i = 0; i < bcnt; i++) {
        ringbuf_push(&h->rx_ring, rx_data[i]);
      }

      /* Notify application */
      if (h->rx_callback) {
        h->rx_callback(h->cb_context, rx_data, bcnt);
      }
    } else if (ep_num == 0) {
      /* Control endpoint — handle SETUP packets for CDC class requests */
      uint8_t setup_data[8];
      if (pktsts == 0x06 && bcnt == 8) {
        usb_read_packet(setup_data, 8);

        uint8_t bmRequestType = setup_data[0];
        uint8_t bRequest = setup_data[1];

        (void)bmRequestType;

        if (bRequest == CDC_SET_LINE_CODING) {
          /* Host is sending 7 bytes of line coding data */
          /* Will be received in the next OUT data stage */
        } else if (bRequest == CDC_GET_LINE_CODING) {
          /* Send current line coding to host */
          usb_write_packet(0, (uint8_t *)&h->line_coding, 7);
        } else if (bRequest == CDC_SET_CONTROL_LINE_STATE) {
          uint16_t wValue = (setup_data[3] << 8) | setup_data[2];
          h->dtr = (wValue & 0x01) ? 1 : 0;
          h->rts = (wValue & 0x02) ? 1 : 0;
          h->configured = 1;

          /* Send zero-length status stage */
          usb_write_packet(0, NULL, 0);
        }
      } else {
        /* Discard unexpected packets */
        usb_read_packet(NULL, bcnt);
      }
    } else {
      usb_read_packet(NULL, bcnt);
    }
  }

  (void)usb_device_descriptor;
  (void)usb_config_descriptor;
  (void)usb_string_langid;
}

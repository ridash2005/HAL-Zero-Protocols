/**
 * @file    usb_cdc.h
 * @brief   USB CDC (Communication Device Class) — Virtual COM Port API
 * @version 1.0.0
 *
 * @details Minimal USB CDC ACM (Abstract Control Model) implementation
 *          for STM32F4xx OTG_FS peripheral. This creates a virtual COM
 *          port visible to host PCs, enabling serial communication over
 *          USB without any external UART-to-USB converter chips.
 *
 * Features:
 *          - USB Full-Speed (12 Mbit/s) device
 *          - CDC ACM class (Virtual COM Port)
 *          - Endpoint configuration: EP0 (control), EP1 IN/OUT (bulk data)
 *          - Line coding management (baud, parity, stop bits)
 *          - Transmit and receive API for application data
 *          - Ring buffer for RX data
 *
 * Use Case:
 *          Replace UART + FTDI/CP2104 chip with a single USB connection.
 *          The host PC sees a standard COM port. No driver installation
 *          needed on Windows 10+, Linux, or macOS.
 */

#ifndef USB_CDC_H
#define USB_CDC_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 *  Configuration
 * ═══════════════════════════════════════════════════════════════════════════
 */

#ifndef USB_CDC_RX_BUFFER_SIZE
#define USB_CDC_RX_BUFFER_SIZE 512 /*!< RX ring buffer size        */
#endif

#ifndef USB_CDC_TX_BUFFER_SIZE
#define USB_CDC_TX_BUFFER_SIZE 512 /*!< TX buffer size             */
#endif

#define USB_CDC_PACKET_SIZE 64 /*!< Max packet size (FS bulk)  */

/* ═══════════════════════════════════════════════════════════════════════════
 *  USB Descriptor Constants
 * ═══════════════════════════════════════════════════════════════════════════
 */

/* Class codes */
#define USB_CLASS_CDC 0x02
#define USB_CDC_SUBCLASS_ACM 0x02
#define USB_CDC_PROTOCOL_AT 0x01 /*!< AT commands (V.25ter)     */

/* CDC-specific request codes */
#define CDC_SET_LINE_CODING 0x20
#define CDC_GET_LINE_CODING 0x21
#define CDC_SET_CONTROL_LINE_STATE 0x22
#define CDC_SEND_BREAK 0x23

/* ═══════════════════════════════════════════════════════════════════════════
 *  Line Coding Structure (CDC ACM)
 * ═══════════════════════════════════════════════════════════════════════════
 */

/**
 * @brief  Line coding parameters as defined by USB CDC ACM specification.
 *         These are sent by the host when opening the virtual COM port.
 */
#if defined(_MSC_VER)
#pragma pack(push, 1)
typedef struct {
  uint32_t baud_rate; /*!< Data terminal rate (bps)              */
  uint8_t stop_bits;  /*!< 0=1 stop, 1=1.5 stop, 2=2 stop       */
  uint8_t parity;     /*!< 0=None, 1=Odd, 2=Even, 3=Mark, 4=Space */
  uint8_t data_bits;  /*!< 5, 6, 7, 8, or 16 bits               */
} usb_cdc_line_coding_t;
#pragma pack(pop)
#else
typedef struct __attribute__((packed)) {
  uint32_t baud_rate; /*!< Data terminal rate (bps)              */
  uint8_t stop_bits;  /*!< 0=1 stop, 1=1.5 stop, 2=2 stop       */
  uint8_t parity;     /*!< 0=None, 1=Odd, 2=Even, 3=Mark, 4=Space */
  uint8_t data_bits;  /*!< 5, 6, 7, 8, or 16 bits               */
} usb_cdc_line_coding_t;
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 *  Status / Error Codes
 * ═══════════════════════════════════════════════════════════════════════════
 */

typedef enum {
  USB_CDC_OK = 0,
  USB_CDC_ERROR = -1,
  USB_CDC_BUSY = -2,
  USB_CDC_OVERRUN = -3,
  USB_CDC_DISCONNECTED = -4,
} usb_cdc_status_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Ring Buffer
 * ═══════════════════════════════════════════════════════════════════════════
 */

typedef struct {
  uint8_t buffer[USB_CDC_RX_BUFFER_SIZE];
  volatile uint16_t head;
  volatile uint16_t tail;
} usb_cdc_ringbuf_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  CDC Device Handle
 * ═══════════════════════════════════════════════════════════════════════════
 */

typedef void (*usb_cdc_rx_callback_t)(void *context, const uint8_t *data,
                                      uint16_t len);

typedef struct {
  /* Configuration */
  usb_cdc_line_coding_t line_coding; /*!< Current line coding           */
  uint8_t dtr;                       /*!< Data Terminal Ready (host)     */
  uint8_t rts;                       /*!< Request To Send (host)        */

  /* Buffers */
  usb_cdc_ringbuf_t rx_ring; /*!< Receive ring buffer           */
  uint8_t tx_buffer[USB_CDC_TX_BUFFER_SIZE];
  volatile uint8_t tx_busy; /*!< TX in progress flag           */

  /* State */
  volatile uint8_t connected;  /*!< USB host connected            */
  volatile uint8_t configured; /*!< USB device configured         */

  /* Callbacks */
  usb_cdc_rx_callback_t rx_callback; /*!< Data received callback        */
  void *cb_context;
} usb_cdc_handle_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════════════
 */

/**
 * @brief  Initialize the USB CDC device.
 * @param  h: CDC handle
 * @retval usb_cdc_status_t
 *
 * @note   This configures the USB OTG_FS peripheral in device mode,
 *         sets up the endpoints, and enables USB interrupts.
 */
usb_cdc_status_t USB_CDC_Init(usb_cdc_handle_t *h);

/**
 * @brief  De-initialize and disconnect from USB bus.
 * @param  h: CDC handle
 */
usb_cdc_status_t USB_CDC_DeInit(usb_cdc_handle_t *h);

/**
 * @brief  Transmit data over the virtual COM port.
 * @param  h: CDC handle
 * @param  data: Pointer to data buffer
 * @param  len: Number of bytes to send
 * @retval usb_cdc_status_t
 *
 * @note   Data is queued for transmission. For large blocks, this
 *         function will split into multiple USB packets.
 */
usb_cdc_status_t USB_CDC_Transmit(usb_cdc_handle_t *h, const uint8_t *data,
                                  uint16_t len);

/**
 * @brief  Read available data from the receive buffer.
 * @param  h: CDC handle
 * @param  data: Destination buffer
 * @param  max_len: Maximum bytes to read
 * @retval Number of bytes actually read
 */
uint16_t USB_CDC_Receive(usb_cdc_handle_t *h, uint8_t *data, uint16_t max_len);

/**
 * @brief  Get the number of bytes available for reading.
 * @param  h: CDC handle
 * @retval Number of bytes in the RX ring buffer
 */
uint16_t USB_CDC_Available(usb_cdc_handle_t *h);

/**
 * @brief  Printf-style formatted output over USB CDC.
 * @param  h: CDC handle
 * @param  fmt: Format string
 * @retval Number of bytes sent
 */
int USB_CDC_Printf(usb_cdc_handle_t *h, const char *fmt, ...);

/**
 * @brief  Check if USB host is connected and device is configured.
 * @param  h: CDC handle
 * @retval 1 if connected and ready, 0 otherwise
 */
uint8_t USB_CDC_IsConnected(usb_cdc_handle_t *h);

/**
 * @brief  Get the current line coding (baud rate, etc.) set by the host.
 * @param  h: CDC handle
 * @retval Current line coding settings
 */
usb_cdc_line_coding_t USB_CDC_GetLineCoding(usb_cdc_handle_t *h);

/**
 * @brief  Register a callback for data reception.
 * @param  h: CDC handle
 * @param  cb: Callback function
 * @param  ctx: User context
 */
void USB_CDC_RegisterCallback(usb_cdc_handle_t *h, usb_cdc_rx_callback_t cb,
                              void *ctx);

/**
 * @brief  USB OTG FS global interrupt handler.
 *         Call this from OTG_FS_IRQHandler().
 * @param  h: CDC handle
 */
void USB_CDC_IRQHandler(usb_cdc_handle_t *h);

#ifdef __cplusplus
}
#endif

#endif /* USB_CDC_H */

/**
 * @file    error_codes.h
 * @brief   Unified error codes for all peripheral drivers
 * @version 1.0.0
 */

#ifndef ERROR_CODES_H
#define ERROR_CODES_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Driver return status codes
 */
typedef enum {
  DRV_OK = 0x00,            /*!< Operation completed successfully          */
  DRV_ERROR = 0x01,         /*!< Generic error                             */
  DRV_BUSY = 0x02,          /*!< Peripheral is busy                        */
  DRV_TIMEOUT = 0x03,       /*!< Operation timed out                       */
  DRV_INVALID_PARAM = 0x04, /*!< Invalid parameter passed                  */
  DRV_NOT_READY = 0x05,     /*!< Peripheral not initialized                */
  DRV_OVERRUN = 0x06,       /*!< Data overrun error                        */
  DRV_FRAME_ERROR = 0x07,   /*!< Framing error (UART)                      */
  DRV_PARITY_ERROR = 0x08,  /*!< Parity error (UART)                       */
  DRV_NOISE_ERROR = 0x09,   /*!< Noise detected (UART)                     */
  DRV_ACK_FAIL = 0x0A,      /*!< Acknowledge failure (I2C)                 */
  DRV_ARB_LOST = 0x0B,      /*!< Arbitration lost (I2C/CAN)                */
  DRV_DMA_ERROR = 0x0C,     /*!< DMA transfer error                        */
  DRV_CRC_ERROR = 0x0D,     /*!< CRC mismatch                              */
  DRV_BUS_OFF = 0x0E,       /*!< CAN bus off                               */
  DRV_FIFO_FULL = 0x0F,     /*!< FIFO/Buffer full                          */
  DRV_FIFO_EMPTY = 0x10,    /*!< FIFO/Buffer empty                         */
  DRV_NOT_SUPPORTED = 0x11, /*!< Feature not supported on this variant     */
} drv_status_t;

/**
 * @brief Peripheral transfer state
 */
typedef enum {
  XFER_STATE_RESET = 0x00,     /*!< Peripheral not configured                 */
  XFER_STATE_READY = 0x01,     /*!< Ready for new transfer                    */
  XFER_STATE_BUSY_TX = 0x02,   /*!< Transmit in progress                      */
  XFER_STATE_BUSY_RX = 0x03,   /*!< Receive in progress                       */
  XFER_STATE_BUSY_TXRX = 0x04, /*!< Full-duplex transfer in progress */
  XFER_STATE_ERROR = 0x05,     /*!< Error state                               */
  XFER_STATE_ABORT = 0x06,     /*!< Transfer aborted                          */
} xfer_state_t;

/**
 * @brief Callback event types
 */
typedef enum {
  EVT_TX_COMPLETE = 0x00,   /*!< Transmission complete                     */
  EVT_RX_COMPLETE = 0x01,   /*!< Reception complete                        */
  EVT_TXRX_COMPLETE = 0x02, /*!< Full-duplex transfer complete              */
  EVT_HALF_COMPLETE = 0x03, /*!< Half transfer complete (DMA)              */
  EVT_ERROR = 0x04,         /*!< Error event                               */
  EVT_IDLE = 0x05,          /*!< Line idle detected                        */
  EVT_ABORT = 0x06,         /*!< Transfer aborted                          */
} callback_event_t;

#ifdef __cplusplus
}
#endif

#endif /* ERROR_CODES_H */

/**
 * @file    can_driver.h
 * @brief   STM32F4xx CAN Bus (bxCAN) Driver — Public API
 * @version 1.0.0
 *
 * @details CAN 2.0B driver for the bxCAN peripheral found in STM32F4xx.
 *
 * Features:
 *          - Standard (11-bit) and Extended (29-bit) identifiers
 *          - Configurable bit timing (125K, 250K, 500K, 1M baud)
 *          - 14 filter banks with ID Mask / ID List modes
 *          - 3 transmit mailboxes, 2 receive FIFOs
 *          - Interrupt-driven reception with callbacks
 *          - Loopback and Silent modes for testing
 *
 * Real-World Context (Automotive / Bosch):
 *          CAN Bus is the backbone of all modern vehicles. An ECU
 *          (Electronic Control Unit) uses CAN to communicate with
 *          other nodes on the vehicle bus (ABS, airbags, engine, etc.).
 */

#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include "../../drivers/common/error_codes.h"
#include "../../drivers/common/stm32f4xx_base.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 *  bxCAN Register Bit Definitions (subset)
 * ═══════════════════════════════════════════════════════════════════════════
 */

/* MCR */
#define CAN_MCR_INRQ (1 << 0)  /*!< Initialization request    */
#define CAN_MCR_SLEEP (1 << 1) /*!< Sleep mode request        */
#define CAN_MCR_TXFP (1 << 2)  /*!< Transmit FIFO priority    */
#define CAN_MCR_RFLM (1 << 3)  /*!< Receive FIFO locked       */
#define CAN_MCR_NART (1 << 4)  /*!< No auto retransmission    */
#define CAN_MCR_AWUM (1 << 5)  /*!< Auto wakeup              */
#define CAN_MCR_ABOM (1 << 6)  /*!< Auto bus-off management   */

/* MSR */
#define CAN_MSR_INAK (1 << 0) /*!< Initialization ack        */
#define CAN_MSR_SLAK (1 << 1) /*!< Sleep ack                */

/* BTR */
#define CAN_BTR_LBKM (1 << 30) /*!< Loopback mode            */
#define CAN_BTR_SILM (1 << 31) /*!< Silent mode              */

/* TSR — Transmit Status */
#define CAN_TSR_RQCP0 (1 << 0)
#define CAN_TSR_TXOK0 (1 << 1)
#define CAN_TSR_TME0 (1 << 26)
#define CAN_TSR_TME1 (1 << 27)
#define CAN_TSR_TME2 (1 << 28)

/* RF0R — Receive FIFO 0 */
#define CAN_RF0R_FMP0_MASK (0x3) /*!< FIFO 0 message pending    */
#define CAN_RF0R_RFOM0 (1 << 5)  /*!< Release FIFO 0 output MB  */

/* IER — Interrupt Enable */
#define CAN_IER_TMEIE (1 << 0)  /*!< Transmit MB empty IE      */
#define CAN_IER_FMPIE0 (1 << 1) /*!< FIFO 0 message pending IE */
#define CAN_IER_FMPIE1 (1 << 4) /*!< FIFO 1 message pending IE */
#define CAN_IER_ERRIE (1 << 15) /*!< Error interrupt enable    */
#define CAN_IER_BOFIE (1 << 10) /*!< Bus-off interrupt enable  */

/* TI0R — TX Mailbox Identifier */
#define CAN_TIR_TXRQ (1 << 0) /*!< Transmit request          */
#define CAN_TIR_RTR (1 << 1)  /*!< Remote transmission req.  */
#define CAN_TIR_IDE (1 << 2)  /*!< Identifier extension      */

/* RI0R — RX FIFO Identifier */
#define CAN_RIR_RTR (1 << 1)
#define CAN_RIR_IDE (1 << 2)

/* ═══════════════════════════════════════════════════════════════════════════
 *  Configuration Enumerations
 * ═══════════════════════════════════════════════════════════════════════════
 */

typedef enum {
  CAN_MODE_NORMAL = 0x00,
  CAN_MODE_LOOPBACK = 0x01,
  CAN_MODE_SILENT = 0x02,
  CAN_MODE_SILENT_LB = 0x03 /*!< Silent + Loopback (self-test) */
} can_mode_t;

typedef enum {
  CAN_SPEED_125K = 125000,
  CAN_SPEED_250K = 250000,
  CAN_SPEED_500K = 500000,
  CAN_SPEED_1M = 1000000
} can_speed_t;

typedef enum {
  CAN_ID_STD = 0, /*!< Standard 11-bit identifier        */
  CAN_ID_EXT = 1  /*!< Extended 29-bit identifier        */
} can_id_type_t;

typedef enum { CAN_RTR_DATA = 0, CAN_RTR_REMOTE = 1 } can_rtr_t;

typedef enum {
  CAN_FILTER_MASK = 0, /*!< ID mask mode (ID & mask = filter) */
  CAN_FILTER_LIST = 1  /*!< ID list mode (exact match)        */
} can_filter_mode_t;

typedef enum {
  CAN_FILTER_SCALE_16 = 0, /*!< Two 16-bit filters                */
  CAN_FILTER_SCALE_32 = 1  /*!< One 32-bit filter                 */
} can_filter_scale_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Message Structures
 * ═══════════════════════════════════════════════════════════════════════════
 */

typedef struct {
  uint32_t id;           /*!< Identifier (11 or 29 bits)        */
  can_id_type_t id_type; /*!< Standard or Extended              */
  can_rtr_t rtr;         /*!< Data or Remote frame              */
  uint8_t dlc;           /*!< Data length code (0-8)            */
  uint8_t data[8];       /*!< Payload data                      */
} CAN_TxMessage_t;

typedef struct {
  uint32_t id;
  can_id_type_t id_type;
  can_rtr_t rtr;
  uint8_t dlc;
  uint8_t data[8];
  uint8_t filter_match; /*!< Which filter matched              */
  uint16_t timestamp;   /*!< Receive timestamp                 */
} CAN_RxMessage_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Handle Structures
 * ═══════════════════════════════════════════════════════════════════════════
 */

typedef void (*can_rx_callback_t)(void *context, CAN_RxMessage_t *msg);
typedef void (*can_tx_callback_t)(void *context);
typedef void (*can_error_callback_t)(void *context, uint32_t error_code);

typedef struct {
  can_mode_t mode;          /*!< Operating mode                    */
  uint32_t baud_rate;       /*!< Bit rate in bps                   */
  uint8_t auto_bus_off;     /*!< Automatic bus-off management      */
  uint8_t auto_retransmit;  /*!< Enable auto retransmission       */
  uint8_t auto_wakeup;      /*!< Automatic wakeup from sleep       */
  uint8_t tx_fifo_priority; /*!< TX FIFO priority (chronological)*/
} CAN_Config_t;

typedef struct {
  uint8_t filter_number;    /*!< Filter bank number (0-13)          */
  can_filter_mode_t mode;   /*!< Mask or List mode                  */
  can_filter_scale_t scale; /*!< 16-bit or 32-bit                   */
  uint32_t id_high;         /*!< Filter ID (high or 1st 16-bit)     */
  uint32_t id_low;          /*!< Filter ID (low or 2nd 16-bit)      */
  uint32_t mask_high;       /*!< Mask (high) or 2nd ID in list mode */
  uint32_t mask_low;        /*!< Mask (low) or 4th filter in list   */
  uint8_t fifo_assignment;  /*!< FIFO 0 or FIFO 1                  */
  uint8_t active;           /*!< Filter activation                  */
} CAN_Filter_t;

typedef struct {
  CAN_RegDef_t *pCANx; /*!< CAN peripheral base address        */
  CAN_Config_t config; /*!< Configuration                      */
  can_rx_callback_t rx_callback;
  can_tx_callback_t tx_callback;
  can_error_callback_t error_callback;
  void *rx_cb_context;    /*!< RX callback context               */
  void *tx_cb_context;    /*!< TX callback context               */
  void *error_cb_context; /*!< Error callback context            */
} CAN_Handle_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════════════
 */

/* Initialization */
drv_status_t CAN_Init(CAN_Handle_t *pHandle);
drv_status_t CAN_DeInit(CAN_RegDef_t *pCANx);
void CAN_PeriClockControl(CAN_RegDef_t *pCANx, uint8_t en);

/* Filter Configuration */
drv_status_t CAN_ConfigFilter(CAN_Handle_t *pHandle, CAN_Filter_t *filter);

/* Transmission */
drv_status_t CAN_Transmit(CAN_Handle_t *pHandle, CAN_TxMessage_t *msg,
                          uint32_t timeout);
drv_status_t CAN_Transmit_IT(CAN_Handle_t *pHandle, CAN_TxMessage_t *msg);

/* Reception */
drv_status_t CAN_Receive(CAN_Handle_t *pHandle, uint8_t fifo,
                         CAN_RxMessage_t *msg, uint32_t timeout);
void CAN_EnableRxInterrupt(CAN_Handle_t *pHandle, uint8_t fifo);

/* IRQ Handlers */
void CAN_TX_IRQHandler(CAN_Handle_t *pHandle);
void CAN_RX0_IRQHandler(CAN_Handle_t *pHandle);
void CAN_RX1_IRQHandler(CAN_Handle_t *pHandle);
void CAN_SCE_IRQHandler(CAN_Handle_t *pHandle);

/* Control */
drv_status_t CAN_Start(CAN_Handle_t *pHandle);
drv_status_t CAN_Stop(CAN_Handle_t *pHandle);
drv_status_t CAN_EnterSleepMode(CAN_Handle_t *pHandle);

/* Status */
uint8_t CAN_GetTxMailboxFreeCount(CAN_Handle_t *pHandle);
uint8_t CAN_GetRxFifoLevel(CAN_Handle_t *pHandle, uint8_t fifo);
uint32_t CAN_GetErrorCode(CAN_Handle_t *pHandle);

/* Callbacks */
void CAN_RegisterRxCallback(CAN_Handle_t *pHandle, can_rx_callback_t cb,
                            void *ctx);
void CAN_RegisterTxCallback(CAN_Handle_t *pHandle, can_tx_callback_t cb,
                            void *ctx);
void CAN_RegisterErrorCallback(CAN_Handle_t *pHandle, can_error_callback_t cb,
                               void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* CAN_DRIVER_H */

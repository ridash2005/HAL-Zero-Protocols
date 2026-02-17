/**
 * @file    uart_driver.h
 * @brief   STM32F4xx UART/USART Driver — Public API
 * @version 2.0.0
 *
 * @details Complete UART driver with three transfer modes:
 *          1. Polling (blocking)     — simple, waits for completion
 *          2. Interrupt (non-blocking) — ISR-based with callbacks
 *          3. DMA (non-blocking)     — zero-CPU-overhead transfers
 *
 *          Supports: USART1/2/3, UART4/5, USART6
 */

#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include "../common/error_codes.h"
#include "../common/stm32f4xx_base.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 *  Configuration Enumerations
 * ═══════════════════════════════════════════════════════════════════════════
 */

/** @brief Standard baud rates */
typedef enum {
  UART_BAUD_1200 = 1200,
  UART_BAUD_2400 = 2400,
  UART_BAUD_9600 = 9600,
  UART_BAUD_19200 = 19200,
  UART_BAUD_38400 = 38400,
  UART_BAUD_57600 = 57600,
  UART_BAUD_115200 = 115200,
  UART_BAUD_230400 = 230400,
  UART_BAUD_460800 = 460800,
  UART_BAUD_921600 = 921600
} uart_baud_t;

/** @brief Word length */
typedef enum {
  UART_WORDLEN_8BIT = 0, /*!< 8 data bits */
  UART_WORDLEN_9BIT = 1  /*!< 9 data bits */
} uart_wordlen_t;

/** @brief Stop bits */
typedef enum {
  UART_STOPBITS_1 = 0x00,
  UART_STOPBITS_0_5 = 0x01,
  UART_STOPBITS_2 = 0x02,
  UART_STOPBITS_1_5 = 0x03
} uart_stopbits_t;

/** @brief Parity */
typedef enum {
  UART_PARITY_NONE = 0x00,
  UART_PARITY_EVEN = 0x01,
  UART_PARITY_ODD = 0x02
} uart_parity_t;

/** @brief Transfer direction */
typedef enum {
  UART_MODE_TX = 0x00,
  UART_MODE_RX = 0x01,
  UART_MODE_TXRX = 0x02
} uart_mode_t;

/** @brief Hardware flow control */
typedef enum {
  UART_HW_FLOW_NONE = 0x00,
  UART_HW_FLOW_CTS = 0x01,
  UART_HW_FLOW_RTS = 0x02,
  UART_HW_FLOW_CTS_RTS = 0x03
} uart_hwflow_t;

/** @brief Oversampling */
typedef enum {
  UART_OVERSAMPLE_16 = 0,
  UART_OVERSAMPLE_8 = 1
} uart_oversample_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  USART Register Bit Definitions
 * ═══════════════════════════════════════════════════════════════════════════
 */

/* SR register bits */
#define USART_SR_PE (1 << 0)   /*!< Parity error              */
#define USART_SR_FE (1 << 1)   /*!< Framing error             */
#define USART_SR_NF (1 << 2)   /*!< Noise detected flag       */
#define USART_SR_ORE (1 << 3)  /*!< Overrun error             */
#define USART_SR_IDLE (1 << 4) /*!< IDLE line detected        */
#define USART_SR_RXNE (1 << 5) /*!< Read data register not empty */
#define USART_SR_TC (1 << 6)   /*!< Transmission complete     */
#define USART_SR_TXE (1 << 7)  /*!< Transmit data register empty */
#define USART_SR_LBD (1 << 8)  /*!< LIN break detection flag  */
#define USART_SR_CTS (1 << 9)  /*!< CTS flag                  */

/* CR1 register bits */
#define USART_CR1_SBK (1 << 0)
#define USART_CR1_RWU (1 << 1)
#define USART_CR1_RE (1 << 2)     /*!< Receiver enable           */
#define USART_CR1_TE (1 << 3)     /*!< Transmitter enable        */
#define USART_CR1_IDLEIE (1 << 4) /*!< IDLE interrupt enable     */
#define USART_CR1_RXNEIE (1 << 5) /*!< RXNE interrupt enable     */
#define USART_CR1_TCIE (1 << 6)   /*!< TC interrupt enable       */
#define USART_CR1_TXEIE (1 << 7)  /*!< TXE interrupt enable      */
#define USART_CR1_PEIE (1 << 8)   /*!< PE interrupt enable       */
#define USART_CR1_PS (1 << 9)     /*!< Parity selection (0=Even) */
#define USART_CR1_PCE (1 << 10)   /*!< Parity control enable     */
#define USART_CR1_WAKE (1 << 11)
#define USART_CR1_M (1 << 12)     /*!< Word length (0=8, 1=9)    */
#define USART_CR1_UE (1 << 13)    /*!< USART enable              */
#define USART_CR1_OVER8 (1 << 15) /*!< Oversampling mode         */

/* CR2 register bits */
#define USART_CR2_STOP_MASK (0x3 << 12)

/* CR3 register bits */
#define USART_CR3_EIE (1 << 0) /*!< Error interrupt enable    */
#define USART_CR3_IREN (1 << 1)
#define USART_CR3_IRLP (1 << 2)
#define USART_CR3_HDSEL (1 << 3)
#define USART_CR3_NACK (1 << 4)
#define USART_CR3_SCEN (1 << 5)
#define USART_CR3_DMAR (1 << 6) /*!< DMA enable receiver       */
#define USART_CR3_DMAT (1 << 7) /*!< DMA enable transmitter    */
#define USART_CR3_RTSE (1 << 8) /*!< RTS enable                */
#define USART_CR3_CTSE (1 << 9) /*!< CTS enable                */
#define USART_CR3_CTSIE (1 << 10)

/* ═══════════════════════════════════════════════════════════════════════════
 *  Handle / Callback Types
 * ═══════════════════════════════════════════════════════════════════════════
 */

/** @brief Application callback function pointer */
typedef void (*uart_callback_t)(void *context, callback_event_t event);

/** @brief UART configuration structure */
typedef struct {
  uint32_t baud_rate;             /*!< Baud rate value                   */
  uart_wordlen_t word_length;     /*!< Data word length                  */
  uart_stopbits_t stop_bits;      /*!< Number of stop bits               */
  uart_parity_t parity;           /*!< Parity mode                       */
  uart_mode_t mode;               /*!< TX / RX / TX+RX                   */
  uart_hwflow_t hw_flow_control;  /*!< Hardware flow control             */
  uart_oversample_t oversampling; /*!< Oversampling mode                 */
} UART_Config_t;

/** @brief UART handle (state + hardware reference) */
typedef struct {
  USART_RegDef_t *pUSARTx; /*!< USART peripheral base address     */
  UART_Config_t config;    /*!< Configuration settings            */

  /* Transfer state (used by interrupt/DMA modes) */
  uint8_t *pTxBuffer;             /*!< Pointer to TX data buffer         */
  uint8_t *pRxBuffer;             /*!< Pointer to RX data buffer         */
  uint32_t tx_len;                /*!< TX transfer length                */
  uint32_t rx_len;                /*!< RX transfer length                */
  volatile uint32_t tx_count;     /*!< Bytes transmitted so far          */
  volatile uint32_t rx_count;     /*!< Bytes received so far             */
  volatile xfer_state_t tx_state; /*!< TX transfer state                 */
  volatile xfer_state_t rx_state; /*!< RX transfer state                 */

  /* Callback */
  uart_callback_t callback; /*!< Application event callback        */
  void *cb_context;         /*!< User context passed to callback   */
} UART_Handle_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API — Initialization
 * ═══════════════════════════════════════════════════════════════════════════
 */

drv_status_t UART_Init(UART_Handle_t *pHandle);
drv_status_t UART_DeInit(USART_RegDef_t *pUSARTx);
void UART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t en);

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API — Polling (Blocking)
 * ═══════════════════════════════════════════════════════════════════════════
 */

drv_status_t UART_Transmit(UART_Handle_t *pHandle, uint8_t *pData,
                           uint32_t len);
drv_status_t UART_Receive(UART_Handle_t *pHandle, uint8_t *pData, uint32_t len);

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API — Interrupt (Non-Blocking)
 * ═══════════════════════════════════════════════════════════════════════════
 */

drv_status_t UART_Transmit_IT(UART_Handle_t *pHandle, uint8_t *pData,
                              uint32_t len);
drv_status_t UART_Receive_IT(UART_Handle_t *pHandle, uint8_t *pData,
                             uint32_t len);
void UART_IRQHandler(UART_Handle_t *pHandle);

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API — DMA (Non-Blocking)
 * ═══════════════════════════════════════════════════════════════════════════
 */

drv_status_t UART_Transmit_DMA(UART_Handle_t *pHandle, uint8_t *pData,
                               uint32_t len);
drv_status_t UART_Receive_DMA(UART_Handle_t *pHandle, uint8_t *pData,
                              uint32_t len);

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API — Control & Status
 * ═══════════════════════════════════════════════════════════════════════════
 */

void UART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t en);
uint8_t UART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t flag);
void UART_ClearFlag(USART_RegDef_t *pUSARTx, uint32_t flag);
void UART_RegisterCallback(UART_Handle_t *pHandle, uart_callback_t cb,
                           void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* UART_DRIVER_H */

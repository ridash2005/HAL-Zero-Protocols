/**
 * @file    i2c_driver.h
 * @brief   STM32F4xx I2C Driver — Public API
 * @version 2.0.0
 *
 * @details I2C Master driver supporting:
 *          - Standard Mode (100 kHz) and Fast Mode (400 kHz)
 *          - 7-bit addressing
 *          - Master Transmit / Receive (blocking and interrupt)
 *          - Repeated START for combined transactions
 *          - ACK control and error detection
 */

#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#include "../common/error_codes.h"
#include "../common/stm32f4xx_base.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 *  I2C Register Bit Definitions
 * ═══════════════════════════════════════════════════════════════════════════
 */

/* CR1 */
#define I2C_CR1_PE (1 << 0)
#define I2C_CR1_SMBUS (1 << 1)
#define I2C_CR1_SMBTYPE (1 << 3)
#define I2C_CR1_ENARP (1 << 4)
#define I2C_CR1_ENPEC (1 << 5)
#define I2C_CR1_ENGC (1 << 6)
#define I2C_CR1_NOSTRETCH (1 << 7)
#define I2C_CR1_START (1 << 8)
#define I2C_CR1_STOP (1 << 9)
#define I2C_CR1_ACK (1 << 10)
#define I2C_CR1_POS (1 << 11)
#define I2C_CR1_PEC (1 << 12)
#define I2C_CR1_ALERT (1 << 13)
#define I2C_CR1_SWRST (1 << 15)

/* CR2 */
#define I2C_CR2_FREQ_MASK 0x3F
#define I2C_CR2_ITERREN (1 << 8)
#define I2C_CR2_ITEVTEN (1 << 9)
#define I2C_CR2_ITBUFEN (1 << 10)
#define I2C_CR2_DMAEN (1 << 11)
#define I2C_CR2_LAST (1 << 12)

/* SR1 */
#define I2C_SR1_SB (1 << 0)   /*!< Start bit sent            */
#define I2C_SR1_ADDR (1 << 1) /*!< Address sent/matched      */
#define I2C_SR1_BTF (1 << 2)  /*!< Byte transfer finished    */
#define I2C_SR1_ADD10 (1 << 3)
#define I2C_SR1_STOPF (1 << 4)
#define I2C_SR1_RXNE (1 << 6)
#define I2C_SR1_TXE (1 << 7)
#define I2C_SR1_BERR (1 << 8) /*!< Bus error                 */
#define I2C_SR1_ARLO (1 << 9) /*!< Arbitration lost          */
#define I2C_SR1_AF (1 << 10)  /*!< Acknowledge failure       */
#define I2C_SR1_OVR (1 << 11)
#define I2C_SR1_PECERR (1 << 12)
#define I2C_SR1_TIMEOUT (1 << 14)

/* SR2 */
#define I2C_SR2_MSL (1 << 0)
#define I2C_SR2_BUSY (1 << 1)
#define I2C_SR2_TRA (1 << 2)

/* CCR */
#define I2C_CCR_FS (1 << 15)   /*!< Fast mode select          */
#define I2C_CCR_DUTY (1 << 14) /*!< Duty cycle (Tlow/Thigh)   */

/* ═══════════════════════════════════════════════════════════════════════════
 *  Configuration Enumerations
 * ═══════════════════════════════════════════════════════════════════════════
 */

typedef enum {
  I2C_SCL_SPEED_SM = 100000,   /*!< Standard mode (100 kHz)   */
  I2C_SCL_SPEED_FM4K = 400000, /*!< Fast mode (400 kHz)       */
  I2C_SCL_SPEED_FM2K = 200000  /*!< Fast mode (200 kHz)       */
} i2c_speed_t;

typedef enum { I2C_ACK_ENABLE = 1, I2C_ACK_DISABLE = 0 } i2c_ack_t;

typedef enum {
  I2C_FM_DUTY_2 = 0,   /*!< Tlow/Thigh = 2            */
  I2C_FM_DUTY_16_9 = 1 /*!< Tlow/Thigh = 16/9         */
} i2c_duty_t;

/** @brief Repeated START control */
typedef enum {
  I2C_REPEATED_START_DISABLE = 0,
  I2C_REPEATED_START_ENABLE = 1
} i2c_rs_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Handle Structures
 * ═══════════════════════════════════════════════════════════════════════════
 */

typedef void (*i2c_callback_t)(void *context, callback_event_t event);

typedef struct {
  uint32_t scl_speed;       /*!< SCL clock speed                   */
  uint8_t device_address;   /*!< Own address (slave mode)          */
  i2c_ack_t ack_control;    /*!< ACK enable/disable                */
  i2c_duty_t fm_duty_cycle; /*!< Fast mode duty cycle              */
} I2C_Config_t;

typedef struct {
  I2C_RegDef_t *pI2Cx; /*!< I2C peripheral base address       */
  I2C_Config_t config; /*!< Configuration                     */

  /* Interrupt transfer state */
  uint8_t *pTxBuffer;
  uint8_t *pRxBuffer;
  uint32_t tx_len;
  uint32_t rx_len;
  uint32_t tx_rx_size; /*!< Total transfer size               */
  volatile xfer_state_t state;
  uint8_t slave_addr;      /*!< Target slave address              */
  i2c_rs_t repeated_start; /*!< Repeated START flag               */

  i2c_callback_t callback;
  void *cb_context;
} I2C_Handle_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════════════
 */

/* Initialization */
drv_status_t I2C_Init(I2C_Handle_t *pHandle);
drv_status_t I2C_DeInit(I2C_RegDef_t *pI2Cx);
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t en);

/* Polling (Blocking) — Master Mode */
drv_status_t I2C_MasterTransmit(I2C_Handle_t *pHandle, uint8_t slave_addr,
                                uint8_t *pData, uint32_t len, i2c_rs_t rs);
drv_status_t I2C_MasterReceive(I2C_Handle_t *pHandle, uint8_t slave_addr,
                               uint8_t *pData, uint32_t len, i2c_rs_t rs);

/* Interrupt (Non-Blocking) — Master Mode */
drv_status_t I2C_MasterTransmit_IT(I2C_Handle_t *pHandle, uint8_t slave_addr,
                                   uint8_t *pData, uint32_t len, i2c_rs_t rs);
drv_status_t I2C_MasterReceive_IT(I2C_Handle_t *pHandle, uint8_t slave_addr,
                                  uint8_t *pData, uint32_t len, i2c_rs_t rs);

/* ISR Handlers (call from I2Cx_EV_IRQHandler and I2Cx_ER_IRQHandler) */
void I2C_EV_IRQHandler(I2C_Handle_t *pHandle);
void I2C_ER_IRQHandler(I2C_Handle_t *pHandle);

/* Control */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t en);
void I2C_ManageACK(I2C_RegDef_t *pI2Cx, uint8_t en);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flag);
void I2C_RegisterCallback(I2C_Handle_t *pHandle, i2c_callback_t cb, void *ctx);
void I2C_CloseSendData(I2C_Handle_t *pHandle);
void I2C_CloseReceiveData(I2C_Handle_t *pHandle);

#ifdef __cplusplus
}
#endif

#endif /* I2C_DRIVER_H */

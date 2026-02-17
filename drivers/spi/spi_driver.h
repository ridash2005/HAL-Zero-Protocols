/**
 * @file    spi_driver.h
 * @brief   STM32F4xx SPI Driver — Public API (Master + Slave)
 * @version 2.0.0
 *
 * @details Full-featured SPI driver supporting:
 *          - Master and Slave modes
 *          - All 4 clock polarity/phase combinations (Mode 0-3)
 *          - 8-bit and 16-bit data frames
 *          - Software and Hardware NSS management
 *          - Polling (blocking) and Interrupt (non-blocking) transfers
 *          - Full-duplex and simplex communication
 */

#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#include "../common/error_codes.h"
#include "../common/stm32f4xx_base.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 *  SPI Register Bit Definitions
 * ═══════════════════════════════════════════════════════════════════════════
 */

/* CR1 */
#define SPI_CR1_CPHA (1 << 0)
#define SPI_CR1_CPOL (1 << 1)
#define SPI_CR1_MSTR (1 << 2)
#define SPI_CR1_BR_MASK (0x7 << 3)
#define SPI_CR1_SPE (1 << 6)
#define SPI_CR1_LSBFIRST (1 << 7)
#define SPI_CR1_SSI (1 << 8)
#define SPI_CR1_SSM (1 << 9)
#define SPI_CR1_RXONLY (1 << 10)
#define SPI_CR1_DFF (1 << 11)
#define SPI_CR1_CRCNEXT (1 << 12)
#define SPI_CR1_CRCEN (1 << 13)
#define SPI_CR1_BIDIOE (1 << 14)
#define SPI_CR1_BIDIMODE (1 << 15)

/* CR2 */
#define SPI_CR2_RXDMAEN (1 << 0)
#define SPI_CR2_TXDMAEN (1 << 1)
#define SPI_CR2_SSOE (1 << 2)
#define SPI_CR2_FRF (1 << 4)
#define SPI_CR2_ERRIE (1 << 5)
#define SPI_CR2_RXNEIE (1 << 6)
#define SPI_CR2_TXEIE (1 << 7)

/* SR */
#define SPI_SR_RXNE (1 << 0)
#define SPI_SR_TXE (1 << 1)
#define SPI_SR_CHSIDE (1 << 2)
#define SPI_SR_UDR (1 << 3)
#define SPI_SR_CRCERR (1 << 4)
#define SPI_SR_MODF (1 << 5)
#define SPI_SR_OVR (1 << 6)
#define SPI_SR_BSY (1 << 7)
#define SPI_SR_FRE (1 << 8)

/* ═══════════════════════════════════════════════════════════════════════════
 *  Configuration Enumerations
 * ═══════════════════════════════════════════════════════════════════════════
 */

typedef enum {
  SPI_DEVICE_MODE_SLAVE = 0,
  SPI_DEVICE_MODE_MASTER = 1
} spi_device_mode_t;

typedef enum {
  SPI_BUS_FULLDUPLEX = 0,    /*!< Full-duplex (2-line)          */
  SPI_BUS_HALFDUPLEX = 1,    /*!< Half-duplex (1-line bidi)     */
  SPI_BUS_SIMPLEX_RXONLY = 2 /*!< Simplex receive only          */
} spi_bus_config_t;

typedef enum {
  SPI_SCLK_DIV2 = 0,
  SPI_SCLK_DIV4 = 1,
  SPI_SCLK_DIV8 = 2,
  SPI_SCLK_DIV16 = 3,
  SPI_SCLK_DIV32 = 4,
  SPI_SCLK_DIV64 = 5,
  SPI_SCLK_DIV128 = 6,
  SPI_SCLK_DIV256 = 7
} spi_sclk_speed_t;

typedef enum { SPI_DFF_8BIT = 0, SPI_DFF_16BIT = 1 } spi_dff_t;

typedef enum { SPI_CPOL_LOW = 0, SPI_CPOL_HIGH = 1 } spi_cpol_t;

typedef enum {
  SPI_CPHA_1EDGE = 0, /*!< Data captured on first edge   */
  SPI_CPHA_2EDGE = 1  /*!< Data captured on second edge   */
} spi_cpha_t;

typedef enum {
  SPI_SSM_DISABLE = 0, /*!< Hardware NSS management        */
  SPI_SSM_ENABLE = 1   /*!< Software NSS management        */
} spi_ssm_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Handle Structures
 * ═══════════════════════════════════════════════════════════════════════════
 */

typedef void (*spi_callback_t)(void *context, callback_event_t event);

typedef struct {
  spi_device_mode_t device_mode;
  spi_bus_config_t bus_config;
  spi_sclk_speed_t sclk_speed;
  spi_dff_t dff;
  spi_cpol_t cpol;
  spi_cpha_t cpha;
  spi_ssm_t ssm;
} SPI_Config_t;

typedef struct {
  SPI_RegDef_t *pSPIx; /*!< SPI peripheral base address       */
  SPI_Config_t config; /*!< Configuration                     */

  /* Interrupt transfer state */
  uint8_t *pTxBuffer;
  uint8_t *pRxBuffer;
  uint32_t tx_len;
  uint32_t rx_len;
  volatile xfer_state_t tx_state;
  volatile xfer_state_t rx_state;

  spi_callback_t callback;
  void *cb_context;
} SPI_Handle_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════════════
 */

/* Initialization */
drv_status_t SPI_Init(SPI_Handle_t *pHandle);
drv_status_t SPI_DeInit(SPI_RegDef_t *pSPIx);
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t en);

/* Polling (Blocking) */
drv_status_t SPI_Transmit(SPI_Handle_t *pHandle, uint8_t *pData, uint32_t len);
drv_status_t SPI_Receive(SPI_Handle_t *pHandle, uint8_t *pData, uint32_t len);
drv_status_t SPI_TransmitReceive(SPI_Handle_t *pHandle, uint8_t *pTxData,
                                 uint8_t *pRxData, uint32_t len);

/* Interrupt (Non-Blocking) */
drv_status_t SPI_Transmit_IT(SPI_Handle_t *pHandle, uint8_t *pData,
                             uint32_t len);
drv_status_t SPI_Receive_IT(SPI_Handle_t *pHandle, uint8_t *pData,
                            uint32_t len);
void SPI_IRQHandler(SPI_Handle_t *pHandle);

/* Control */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t en);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t en);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t en);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flag);
void SPI_RegisterCallback(SPI_Handle_t *pHandle, spi_callback_t cb, void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* SPI_DRIVER_H */

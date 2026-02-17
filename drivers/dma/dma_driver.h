/**
 * @file    dma_driver.h
 * @brief   STM32F4xx DMA Driver — Public API
 * @version 2.0.0
 *
 * @details DMA controller driver supporting:
 *          - Memory-to-Peripheral transfers
 *          - Peripheral-to-Memory transfers
 *          - Memory-to-Memory transfers
 *          - Circular and Normal modes
 *          - Transfer complete / Half transfer / Error interrupts
 *          - FIFO mode with configurable threshold
 *
 * @note    STM32F4 has DMA1 (8 streams) and DMA2 (8 streams).
 *          Each stream can be configured for a specific channel (0-7).
 */

#ifndef DMA_DRIVER_H
#define DMA_DRIVER_H

#include "../common/error_codes.h"
#include "../common/stm32f4xx_base.h"


#ifdef __cplusplus
extern "C" {
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 *  DMA Stream CR Bit Definitions
 * ═══════════════════════════════════════════════════════════════════════════
 */

#define DMA_CR_EN (1 << 0)             /*!< Stream enable             */
#define DMA_CR_DMEIE (1 << 1)          /*!< Direct mode error IE      */
#define DMA_CR_TEIE (1 << 2)           /*!< Transfer error IE         */
#define DMA_CR_HTIE (1 << 3)           /*!< Half transfer IE          */
#define DMA_CR_TCIE (1 << 4)           /*!< Transfer complete IE      */
#define DMA_CR_PFCTRL (1 << 5)         /*!< Peripheral flow controller */
#define DMA_CR_DIR_MASK (0x3 << 6)     /*!< Data transfer direction   */
#define DMA_CR_CIRC (1 << 8)           /*!< Circular mode             */
#define DMA_CR_PINC (1 << 9)           /*!< Peripheral increment      */
#define DMA_CR_MINC (1 << 10)          /*!< Memory increment          */
#define DMA_CR_PSIZE_MASK (0x3 << 11)  /*!< Peripheral data size      */
#define DMA_CR_MSIZE_MASK (0x3 << 13)  /*!< Memory data size          */
#define DMA_CR_PINCOS (1 << 15)        /*!< Periph incr offset size   */
#define DMA_CR_PL_MASK (0x3 << 16)     /*!< Priority level            */
#define DMA_CR_DBM (1 << 18)           /*!< Double buffer mode        */
#define DMA_CR_CT (1 << 19)            /*!< Current target            */
#define DMA_CR_PBURST_MASK (0x3 << 21) /*!< Peripheral burst          */
#define DMA_CR_MBURST_MASK (0x3 << 23) /*!< Memory burst              */
#define DMA_CR_CHSEL_MASK (0x7 << 25)  /*!< Channel selection         */

/* FCR Bit Definitions */
#define DMA_FCR_FTH_MASK (0x3 << 0) /*!< FIFO threshold selection  */
#define DMA_FCR_DMDIS (1 << 2)      /*!< Direct mode disable       */
#define DMA_FCR_FS_MASK (0x7 << 3)  /*!< FIFO status               */
#define DMA_FCR_FEIE (1 << 7)       /*!< FIFO error interrupt en.  */

/* ═══════════════════════════════════════════════════════════════════════════
 *  Configuration Enumerations
 * ═══════════════════════════════════════════════════════════════════════════
 */

typedef enum {
  DMA_DIR_PERIPH_TO_MEM = 0x00,
  DMA_DIR_MEM_TO_PERIPH = 0x01,
  DMA_DIR_MEM_TO_MEM = 0x02
} dma_direction_t;

typedef enum {
  DMA_DATASIZE_BYTE = 0x00,
  DMA_DATASIZE_HALFWORD = 0x01,
  DMA_DATASIZE_WORD = 0x02
} dma_data_size_t;

typedef enum {
  DMA_PRIORITY_LOW = 0x00,
  DMA_PRIORITY_MEDIUM = 0x01,
  DMA_PRIORITY_HIGH = 0x02,
  DMA_PRIORITY_VERY_HIGH = 0x03
} dma_priority_t;

typedef enum { DMA_MODE_NORMAL = 0, DMA_MODE_CIRCULAR = 1 } dma_mode_t;

typedef enum {
  DMA_FIFO_THRESHOLD_1_4 = 0x00,
  DMA_FIFO_THRESHOLD_1_2 = 0x01,
  DMA_FIFO_THRESHOLD_3_4 = 0x02,
  DMA_FIFO_THRESHOLD_FULL = 0x03
} dma_fifo_threshold_t;

typedef enum {
  DMA_BURST_SINGLE = 0x00,
  DMA_BURST_INCR4 = 0x01,
  DMA_BURST_INCR8 = 0x02,
  DMA_BURST_INCR16 = 0x03
} dma_burst_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Handle Structures
 * ═══════════════════════════════════════════════════════════════════════════
 */

typedef void (*dma_callback_t)(void *context, callback_event_t event);

typedef struct {
  uint8_t channel;                     /*!< DMA channel (0-7)              */
  dma_direction_t direction;           /*!< Transfer direction             */
  dma_data_size_t periph_data_size;    /*!< Peripheral data width          */
  dma_data_size_t mem_data_size;       /*!< Memory data width              */
  uint8_t periph_increment;            /*!< Peripheral address increment   */
  uint8_t mem_increment;               /*!< Memory address increment       */
  dma_mode_t mode;                     /*!< Normal or Circular             */
  dma_priority_t priority;             /*!< Stream priority                */
  uint8_t fifo_mode;                   /*!< 0 = direct mode, 1 = FIFO      */
  dma_fifo_threshold_t fifo_threshold; /*!< FIFO threshold level            */
  dma_burst_t mem_burst;               /*!< Memory burst configuration      */
  dma_burst_t periph_burst;            /*!< Peripheral burst configuration  */
} DMA_Config_t;

typedef struct {
  DMA_RegDef_t *pDMAx;     /*!< DMA controller (DMA1 or DMA2)  */
  uint8_t stream_num;      /*!< Stream number (0-7)            */
  DMA_Config_t config;     /*!< Stream configuration           */
  dma_callback_t callback; /*!< Transfer event callback        */
  void *cb_context;        /*!< User context                   */
} DMA_Handle_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════════════
 */

/* Initialization */
drv_status_t DMA_Init(DMA_Handle_t *pHandle);
drv_status_t DMA_DeInit(DMA_Handle_t *pHandle);
void DMA_PeriClockControl(DMA_RegDef_t *pDMAx, uint8_t en);

/* Transfer Control */
drv_status_t DMA_Start(DMA_Handle_t *pHandle, uint32_t src_addr,
                       uint32_t dst_addr, uint32_t data_count);
drv_status_t DMA_Start_IT(DMA_Handle_t *pHandle, uint32_t src_addr,
                          uint32_t dst_addr, uint32_t data_count);
void DMA_Abort(DMA_Handle_t *pHandle);

/* Polling */
drv_status_t DMA_PollForTransfer(DMA_Handle_t *pHandle, uint32_t timeout);

/* IRQ Handler */
void DMA_IRQHandler(DMA_Handle_t *pHandle);

/* Status */
uint32_t DMA_GetRemainingCount(DMA_Handle_t *pHandle);

/* Callback */
void DMA_RegisterCallback(DMA_Handle_t *pHandle, dma_callback_t cb, void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* DMA_DRIVER_H */

/**
 * @file    dma_driver.c
 * @brief   STM32F4xx DMA Driver — Implementation
 * @version 2.0.0
 *
 * @details Implements DMA stream configuration, transfer start (polling and
 *          interrupt), ISR handling for TC/HT/TE flags, and abort.
 */

#include "dma_driver.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  Internal Helpers — Flag Positions
 * ═══════════════════════════════════════════════════════════════════════════
 */

/*
 * DMA interrupt status and clear registers are organized in a non-trivial way.
 * Streams 0-3 use LISR/LIFCR, Streams 4-7 use HISR/HIFCR.
 * Each stream's flags are at a specific bit offset.
 *
 * Stream 0: bits [0..5],   Stream 1: bits [6..11]
 * Stream 2: bits [16..21], Stream 3: bits [22..27]
 * (Same pattern for HISR with streams 4-7)
 */
static const uint8_t stream_flag_offset[] = {0, 6, 16, 22, 0, 6, 16, 22};

#define DMA_FLAG_FEIF 0x01
#define DMA_FLAG_DMEIF 0x04
#define DMA_FLAG_TEIF 0x08
#define DMA_FLAG_HTIF 0x10
#define DMA_FLAG_TCIF 0x20

/* ═══════════════════════════════════════════════════════════════════════════
 *  Clock Control
 * ═══════════════════════════════════════════════════════════════════════════
 */

void DMA_PeriClockControl(DMA_RegDef_t *pDMAx, uint8_t en) {
  if (en == ENABLE) {
    if (pDMAx == DMA1) {
      DMA1_PCLK_EN();
    } else {
      DMA2_PCLK_EN();
    }
  } else {
    if (pDMAx == DMA1) {
      DMA1_PCLK_DI();
    } else {
      DMA2_PCLK_DI();
    }
  }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Clear Stream Flags
 * ═══════════════════════════════════════════════════════════════════════════
 */

static void DMA_ClearFlags(DMA_RegDef_t *pDMAx, uint8_t stream) {
  uint32_t clear_mask = 0x3D << stream_flag_offset[stream]; /* All flags */

  if (stream < 4) {
    pDMAx->LIFCR = clear_mask;
  } else {
    pDMAx->HIFCR = clear_mask;
  }
}

static uint32_t DMA_GetFlags(DMA_RegDef_t *pDMAx, uint8_t stream) {
  uint32_t isr;
  if (stream < 4) {
    isr = pDMAx->LISR;
  } else {
    isr = pDMAx->HISR;
  }
  return (isr >> stream_flag_offset[stream]) & 0x3F;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Initialization
 * ═══════════════════════════════════════════════════════════════════════════
 */

drv_status_t DMA_Init(DMA_Handle_t *pHandle) {
  if (pHandle == NULL || pHandle->stream_num > 7) {
    return DRV_INVALID_PARAM;
  }

  DMA_RegDef_t *pDMAx = pHandle->pDMAx;
  uint8_t sn = pHandle->stream_num;
  DMA_Config_t *cfg = &pHandle->config;

  /* Enable clock */
  DMA_PeriClockControl(pDMAx, ENABLE);

  /* Disable stream before configuring */
  pDMAx->STREAM[sn].CR &= ~DMA_CR_EN;
  while (pDMAx->STREAM[sn].CR & DMA_CR_EN)
    ; /* Wait until disabled */

  /* Clear all interrupt flags */
  DMA_ClearFlags(pDMAx, sn);

  /* Build CR register value */
  uint32_t cr = 0;

  /* Channel selection */
  cr |= ((uint32_t)(cfg->channel & 0x7) << 25);

  /* Priority */
  cr |= ((uint32_t)cfg->priority << 16);

  /* Data sizes */
  cr |= ((uint32_t)cfg->mem_data_size << 13);
  cr |= ((uint32_t)cfg->periph_data_size << 11);

  /* Increment modes */
  if (cfg->mem_increment)
    cr |= DMA_CR_MINC;
  if (cfg->periph_increment)
    cr |= DMA_CR_PINC;

  /* Transfer direction */
  cr |= ((uint32_t)cfg->direction << 6);

  /* Circular mode */
  if (cfg->mode == DMA_MODE_CIRCULAR)
    cr |= DMA_CR_CIRC;

  /* Burst configuration */
  cr |= ((uint32_t)cfg->mem_burst << 23);
  cr |= ((uint32_t)cfg->periph_burst << 21);

  pDMAx->STREAM[sn].CR = cr;

  /* Configure FIFO */
  uint32_t fcr = 0;
  if (cfg->fifo_mode) {
    fcr |= DMA_FCR_DMDIS; /* Disable direct mode = enable FIFO */
    fcr |= ((uint32_t)cfg->fifo_threshold & 0x3);
  }
  pDMAx->STREAM[sn].FCR = fcr;

  return DRV_OK;
}

drv_status_t DMA_DeInit(DMA_Handle_t *pHandle) {
  if (pHandle == NULL || pHandle->stream_num > 7) {
    return DRV_INVALID_PARAM;
  }

  uint8_t sn = pHandle->stream_num;

  /* Disable stream */
  pHandle->pDMAx->STREAM[sn].CR = 0;
  pHandle->pDMAx->STREAM[sn].NDTR = 0;
  pHandle->pDMAx->STREAM[sn].PAR = 0;
  pHandle->pDMAx->STREAM[sn].M0AR = 0;
  pHandle->pDMAx->STREAM[sn].M1AR = 0;
  pHandle->pDMAx->STREAM[sn].FCR = 0x21; /* Reset value */

  DMA_ClearFlags(pHandle->pDMAx, sn);

  return DRV_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Start Transfer (Polling)
 * ═══════════════════════════════════════════════════════════════════════════
 */

drv_status_t DMA_Start(DMA_Handle_t *pHandle, uint32_t src_addr,
                       uint32_t dst_addr, uint32_t data_count) {
  if (pHandle == NULL || data_count == 0) {
    return DRV_INVALID_PARAM;
  }

  uint8_t sn = pHandle->stream_num;
  DMA_Stream_RegDef_t *stream = &pHandle->pDMAx->STREAM[sn];

  /* Ensure stream is disabled */
  stream->CR &= ~DMA_CR_EN;
  while (stream->CR & DMA_CR_EN)
    ;

  DMA_ClearFlags(pHandle->pDMAx, sn);

  /* Set addresses based on direction */
  if (pHandle->config.direction == DMA_DIR_MEM_TO_PERIPH) {
    stream->PAR = dst_addr;  /* Peripheral (destination) */
    stream->M0AR = src_addr; /* Memory (source) */
  } else {
    stream->PAR = src_addr;  /* Peripheral (source) */
    stream->M0AR = dst_addr; /* Memory (destination) */
  }

  /* Set data count */
  stream->NDTR = data_count;

  /* Enable the stream */
  stream->CR |= DMA_CR_EN;

  return DRV_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Start Transfer (Interrupt-Driven)
 * ═══════════════════════════════════════════════════════════════════════════
 */

drv_status_t DMA_Start_IT(DMA_Handle_t *pHandle, uint32_t src_addr,
                          uint32_t dst_addr, uint32_t data_count) {
  if (pHandle == NULL || data_count == 0) {
    return DRV_INVALID_PARAM;
  }

  uint8_t sn = pHandle->stream_num;
  DMA_Stream_RegDef_t *stream = &pHandle->pDMAx->STREAM[sn];

  /* Ensure stream is disabled */
  stream->CR &= ~DMA_CR_EN;
  while (stream->CR & DMA_CR_EN)
    ;

  DMA_ClearFlags(pHandle->pDMAx, sn);

  /* Set addresses */
  if (pHandle->config.direction == DMA_DIR_MEM_TO_PERIPH) {
    stream->PAR = dst_addr;
    stream->M0AR = src_addr;
  } else {
    stream->PAR = src_addr;
    stream->M0AR = dst_addr;
  }

  stream->NDTR = data_count;

  /* Enable interrupts: TC, HT, TE */
  stream->CR |= (DMA_CR_TCIE | DMA_CR_HTIE | DMA_CR_TEIE | DMA_CR_DMEIE);

  /* Enable the stream */
  stream->CR |= DMA_CR_EN;

  return DRV_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Poll for Transfer Completion
 * ═══════════════════════════════════════════════════════════════════════════
 */

drv_status_t DMA_PollForTransfer(DMA_Handle_t *pHandle, uint32_t timeout) {
  uint8_t sn = pHandle->stream_num;
  uint32_t tick = 0;

  while (tick < timeout) {
    uint32_t flags = DMA_GetFlags(pHandle->pDMAx, sn);

    if (flags & DMA_FLAG_TEIF) {
      DMA_ClearFlags(pHandle->pDMAx, sn);
      return DRV_DMA_ERROR;
    }

    if (flags & DMA_FLAG_TCIF) {
      DMA_ClearFlags(pHandle->pDMAx, sn);
      return DRV_OK;
    }

    tick++;
  }

  return DRV_TIMEOUT;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Abort Transfer
 * ═══════════════════════════════════════════════════════════════════════════
 */

void DMA_Abort(DMA_Handle_t *pHandle) {
  uint8_t sn = pHandle->stream_num;

  /* Disable the stream */
  pHandle->pDMAx->STREAM[sn].CR &= ~DMA_CR_EN;
  while (pHandle->pDMAx->STREAM[sn].CR & DMA_CR_EN)
    ;

  /* Clear all flags */
  DMA_ClearFlags(pHandle->pDMAx, sn);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  IRQ Handler
 * ═══════════════════════════════════════════════════════════════════════════
 */

void DMA_IRQHandler(DMA_Handle_t *pHandle) {
  uint8_t sn = pHandle->stream_num;
  uint32_t flags = DMA_GetFlags(pHandle->pDMAx, sn);

  /* Transfer Complete */
  if (flags & DMA_FLAG_TCIF) {
    DMA_ClearFlags(pHandle->pDMAx, sn);

    /* In normal mode, disable the stream after TC */
    if (!(pHandle->pDMAx->STREAM[sn].CR & DMA_CR_CIRC)) {
      pHandle->pDMAx->STREAM[sn].CR &= ~DMA_CR_EN;
    }

    if (pHandle->callback) {
      pHandle->callback(pHandle->cb_context, EVT_TX_COMPLETE);
    }
    return;
  }

  /* Half Transfer */
  if (flags & DMA_FLAG_HTIF) {
    /* Clear only HT flag */
    uint32_t ht_mask = DMA_FLAG_HTIF << stream_flag_offset[sn];
    if (sn < 4) {
      pHandle->pDMAx->LIFCR = ht_mask;
    } else {
      pHandle->pDMAx->HIFCR = ht_mask;
    }

    if (pHandle->callback) {
      pHandle->callback(pHandle->cb_context, EVT_HALF_COMPLETE);
    }
    return;
  }

  /* Transfer Error */
  if (flags & DMA_FLAG_TEIF) {
    DMA_ClearFlags(pHandle->pDMAx, sn);
    pHandle->pDMAx->STREAM[sn].CR &= ~DMA_CR_EN;

    if (pHandle->callback) {
      pHandle->callback(pHandle->cb_context, EVT_ERROR);
    }
    return;
  }

  /* FIFO Error */
  if (flags & DMA_FLAG_FEIF) {
    uint32_t fe_mask = DMA_FLAG_FEIF << stream_flag_offset[sn];
    if (sn < 4) {
      pHandle->pDMAx->LIFCR = fe_mask;
    } else {
      pHandle->pDMAx->HIFCR = fe_mask;
    }
  }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Status
 * ═══════════════════════════════════════════════════════════════════════════
 */

uint32_t DMA_GetRemainingCount(DMA_Handle_t *pHandle) {
  return pHandle->pDMAx->STREAM[pHandle->stream_num].NDTR;
}

void DMA_RegisterCallback(DMA_Handle_t *pHandle, dma_callback_t cb, void *ctx) {
  pHandle->callback = cb;
  pHandle->cb_context = ctx;
}

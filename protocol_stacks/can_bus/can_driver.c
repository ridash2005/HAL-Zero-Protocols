/**
 * @file    can_driver.c
 * @brief   STM32F4xx CAN Bus (bxCAN) Driver — Implementation
 * @version 1.0.0
 *
 * @details Implements the bxCAN peripheral driver including initialization
 *          (entering/exiting initialization mode), bit timing configuration,
 *          filter bank setup, and message transmit/receive operations.
 */

#include "can_driver.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  Clock Control
 * ═══════════════════════════════════════════════════════════════════════════
 */

void CAN_PeriClockControl(CAN_RegDef_t *pCANx, uint8_t en) {
  if (en == ENABLE) {
    if (pCANx == CAN1) {
      CAN1_PCLK_EN();
    } else if (pCANx == CAN2) {
      CAN2_PCLK_EN();
    }
  } else {
    if (pCANx == CAN1) {
      CAN1_PCLK_DI();
    } else if (pCANx == CAN2) {
      CAN2_PCLK_DI();
    }
  }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Enter / Exit Initialization Mode
 * ═══════════════════════════════════════════════════════════════════════════
 */

static drv_status_t CAN_EnterInitMode(CAN_RegDef_t *pCANx) {
  /* Exit sleep mode first */
  pCANx->MCR &= ~CAN_MCR_SLEEP;

  /* Request initialization mode */
  pCANx->MCR |= CAN_MCR_INRQ;

  /* Wait for INAK flag to confirm */
  uint32_t timeout = 100000;
  while (!(pCANx->MSR & CAN_MSR_INAK) && timeout--)
    ;

  if (timeout == 0)
    return DRV_TIMEOUT;
  return DRV_OK;
}

static drv_status_t CAN_ExitInitMode(CAN_RegDef_t *pCANx) {
  /* Clear initialization request */
  pCANx->MCR &= ~CAN_MCR_INRQ;

  /* Wait for INAK to clear */
  uint32_t timeout = 100000;
  while ((pCANx->MSR & CAN_MSR_INAK) && timeout--)
    ;

  if (timeout == 0)
    return DRV_TIMEOUT;
  return DRV_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Bit Timing Calculation
 * ═══════════════════════════════════════════════════════════════════════════
 */

/**
 * @brief  Configure the CAN bit timing register (BTR).
 *
 * CAN Bit Timing:
 *   bit_time = SYNC_SEG + TS1 + TS2  (in time quanta, tq)
 *   tq = (BRP + 1) / f_APB1
 *   baud_rate = f_APB1 / ((BRP + 1) × bit_time)
 *
 * Typical configurations (assuming APB1 = 42 MHz):
 *   500K: BRP=5, TS1=10, TS2=3 → 14 tq
 *   250K: BRP=11, TS1=10, TS2=3 → 14 tq
 *   125K: BRP=23, TS1=10, TS2=3 → 14 tq
 *   1M:   BRP=2,  TS1=10, TS2=3 → 14 tq
 */
static void CAN_ConfigBitTiming(CAN_RegDef_t *pCANx, uint32_t baud_rate,
                                can_mode_t mode) {
  /*
   * APB1 clock for CAN peripheral bit timing.
   * With HSI (16 MHz) and no AHB/APB prescalers, APB1 = SYSTEM_CLOCK_HZ.
   * For production with PLL (e.g. 168 MHz SYSCLK), APB1 is typically
   * SYSCLK/4 = 42 MHz. Override CAN_APB1_CLOCK_HZ if using a PLL config.
   */
#ifndef CAN_APB1_CLOCK_HZ
#define CAN_APB1_CLOCK_HZ SYSTEM_CLOCK_HZ
#endif
  uint32_t apb1_clk = CAN_APB1_CLOCK_HZ;
  uint32_t total_tq = 14;
  uint32_t brp = (apb1_clk / (baud_rate * total_tq)) - 1;

  uint32_t ts1 = 9; /* TS1 = 10 tq (register value = N-1) */
  uint32_t ts2 = 2; /* TS2 = 3 tq  (register value = N-1) */
  uint32_t sjw = 0; /* SJW = 1 tq  (register value = N-1) */

  uint32_t btr = 0;
  btr |= (sjw & 0x3) << 24;
  btr |= (ts2 & 0x7) << 20;
  btr |= (ts1 & 0xF) << 16;
  btr |= (brp & 0x3FF);

  /* Configure test mode */
  if (mode == CAN_MODE_LOOPBACK || mode == CAN_MODE_SILENT_LB) {
    btr |= CAN_BTR_LBKM;
  }
  if (mode == CAN_MODE_SILENT || mode == CAN_MODE_SILENT_LB) {
    btr |= CAN_BTR_SILM;
  }

  pCANx->BTR = btr;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Initialization
 * ═══════════════════════════════════════════════════════════════════════════
 */

drv_status_t CAN_Init(CAN_Handle_t *pHandle) {
  if (pHandle == NULL)
    return DRV_INVALID_PARAM;

  CAN_RegDef_t *pCANx = pHandle->pCANx;
  CAN_Config_t *cfg = &pHandle->config;

  /* Enable clock */
  CAN_PeriClockControl(pCANx, ENABLE);

  /* Enter initialization mode */
  drv_status_t status = CAN_EnterInitMode(pCANx);
  if (status != DRV_OK)
    return status;

  /* Configure MCR */
  uint32_t mcr = pCANx->MCR & CAN_MCR_INRQ; /* Preserve INRQ bit */

  if (cfg->auto_bus_off)
    mcr |= CAN_MCR_ABOM;
  if (!cfg->auto_retransmit)
    mcr |= CAN_MCR_NART;
  if (cfg->auto_wakeup)
    mcr |= CAN_MCR_AWUM;
  if (cfg->tx_fifo_priority)
    mcr |= CAN_MCR_TXFP;

  pCANx->MCR = mcr;

  /* Configure bit timing */
  CAN_ConfigBitTiming(pCANx, cfg->baud_rate, cfg->mode);

  return DRV_OK;
}

drv_status_t CAN_Start(CAN_Handle_t *pHandle) {
  return CAN_ExitInitMode(pHandle->pCANx);
}

drv_status_t CAN_Stop(CAN_Handle_t *pHandle) {
  return CAN_EnterInitMode(pHandle->pCANx);
}

drv_status_t CAN_DeInit(CAN_RegDef_t *pCANx) {
  CAN_PeriClockControl(pCANx, DISABLE);
  return DRV_OK;
}

drv_status_t CAN_EnterSleepMode(CAN_Handle_t *pHandle) {
  pHandle->pCANx->MCR |= CAN_MCR_SLEEP;

  uint32_t timeout = 100000;
  while (!(pHandle->pCANx->MSR & CAN_MSR_SLAK) && timeout--)
    ;

  if (timeout == 0)
    return DRV_TIMEOUT;
  return DRV_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Filter Configuration
 * ═══════════════════════════════════════════════════════════════════════════
 */

drv_status_t CAN_ConfigFilter(CAN_Handle_t *pHandle, CAN_Filter_t *filter) {
  if (filter == NULL || filter->filter_number > 13) {
    return DRV_INVALID_PARAM;
  }

  CAN_RegDef_t *pCANx = pHandle->pCANx;
  uint32_t filter_bit = (1UL << filter->filter_number);

  /* Enter filter initialization mode */
  pCANx->FMR |= 1; /* FINIT bit */

  /* Deactivate the filter */
  pCANx->FA1R &= ~filter_bit;

  /* Set filter scale */
  if (filter->scale == CAN_FILTER_SCALE_32) {
    pCANx->FS1R |= filter_bit;
  } else {
    pCANx->FS1R &= ~filter_bit;
  }

  /* Set filter mode */
  if (filter->mode == CAN_FILTER_LIST) {
    pCANx->FM1R |= filter_bit;
  } else {
    pCANx->FM1R &= ~filter_bit;
  }

  /* Set filter register values */
  pCANx->FILTER[filter->filter_number].FR1 =
      ((filter->id_high & 0xFFFF) << 16) | (filter->id_low & 0xFFFF);
  pCANx->FILTER[filter->filter_number].FR2 =
      ((filter->mask_high & 0xFFFF) << 16) | (filter->mask_low & 0xFFFF);

  /* FIFO assignment */
  if (filter->fifo_assignment == 1) {
    pCANx->FFA1R |= filter_bit;
  } else {
    pCANx->FFA1R &= ~filter_bit;
  }

  /* Activate filter */
  if (filter->active) {
    pCANx->FA1R |= filter_bit;
  }

  /* Exit filter initialization mode */
  pCANx->FMR &= ~1;

  return DRV_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Transmit
 * ═══════════════════════════════════════════════════════════════════════════
 */

drv_status_t CAN_Transmit(CAN_Handle_t *pHandle, CAN_TxMessage_t *msg,
                          uint32_t timeout) {
  if (pHandle == NULL || msg == NULL || msg->dlc > 8) {
    return DRV_INVALID_PARAM;
  }

  CAN_RegDef_t *pCANx = pHandle->pCANx;

  /* Find a free transmit mailbox (TME bits in TSR) */
  uint8_t mailbox = 0xFF;
  uint32_t tick = 0;

  while (tick < timeout) {
    if (pCANx->TSR & CAN_TSR_TME0) {
      mailbox = 0;
      break;
    } else if (pCANx->TSR & CAN_TSR_TME1) {
      mailbox = 1;
      break;
    } else if (pCANx->TSR & CAN_TSR_TME2) {
      mailbox = 2;
      break;
    }
    tick++;
  }

  if (mailbox == 0xFF)
    return DRV_TIMEOUT;

  /* Set identifier */
  uint32_t tir = 0;
  if (msg->id_type == CAN_ID_EXT) {
    tir = (msg->id << 3) | CAN_TIR_IDE;
  } else {
    tir = (msg->id << 21);
  }
  if (msg->rtr == CAN_RTR_REMOTE) {
    tir |= CAN_TIR_RTR;
  }

  pCANx->TX_MB[mailbox].TIR = tir;

  /* Set DLC */
  pCANx->TX_MB[mailbox].TDTR = (msg->dlc & 0x0F);

  /* Set data */
  pCANx->TX_MB[mailbox].TDLR =
      ((uint32_t)msg->data[3] << 24) | ((uint32_t)msg->data[2] << 16) |
      ((uint32_t)msg->data[1] << 8) | ((uint32_t)msg->data[0]);
  pCANx->TX_MB[mailbox].TDHR =
      ((uint32_t)msg->data[7] << 24) | ((uint32_t)msg->data[6] << 16) |
      ((uint32_t)msg->data[5] << 8) | ((uint32_t)msg->data[4]);

  /* Request transmission */
  pCANx->TX_MB[mailbox].TIR |= CAN_TIR_TXRQ;

  return DRV_OK;
}

drv_status_t CAN_Transmit_IT(CAN_Handle_t *pHandle, CAN_TxMessage_t *msg) {
  /* Send the message (non-blocking — check for free mailbox immediately) */
  drv_status_t status = CAN_Transmit(pHandle, msg, 1);
  if (status != DRV_OK)
    return status;

  /* Enable transmit mailbox empty interrupt */
  pHandle->pCANx->IER |= CAN_IER_TMEIE;

  return DRV_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Receive
 * ═══════════════════════════════════════════════════════════════════════════
 */

drv_status_t CAN_Receive(CAN_Handle_t *pHandle, uint8_t fifo,
                         CAN_RxMessage_t *msg, uint32_t timeout) {
  if (pHandle == NULL || msg == NULL || fifo > 1) {
    return DRV_INVALID_PARAM;
  }

  CAN_RegDef_t *pCANx = pHandle->pCANx;
  uint32_t tick = 0;

  /* Wait for message in FIFO */
  __vo uint32_t *rfr = (fifo == 0) ? &pCANx->RF0R : &pCANx->RF1R;

  while (tick < timeout) {
    if ((*rfr & CAN_RF0R_FMP0_MASK) != 0)
      break;
    tick++;
  }

  if (tick >= timeout)
    return DRV_TIMEOUT;

  /* Read identifier */
  uint32_t rir = pCANx->RX_FIFO[fifo].RIR;
  if (rir & CAN_RIR_IDE) {
    msg->id_type = CAN_ID_EXT;
    msg->id = (rir >> 3) & 0x1FFFFFFF;
  } else {
    msg->id_type = CAN_ID_STD;
    msg->id = (rir >> 21) & 0x7FF;
  }
  msg->rtr = (rir & CAN_RIR_RTR) ? CAN_RTR_REMOTE : CAN_RTR_DATA;

  /* Read DLC */
  msg->dlc = pCANx->RX_FIFO[fifo].RDTR & 0x0F;

  /* Read filter match index and timestamp */
  msg->filter_match = (pCANx->RX_FIFO[fifo].RDTR >> 8) & 0xFF;
  msg->timestamp = (pCANx->RX_FIFO[fifo].RDTR >> 16) & 0xFFFF;

  /* Read data */
  uint32_t rdlr = pCANx->RX_FIFO[fifo].RDLR;
  uint32_t rdhr = pCANx->RX_FIFO[fifo].RDHR;
  msg->data[0] = (uint8_t)(rdlr);
  msg->data[1] = (uint8_t)(rdlr >> 8);
  msg->data[2] = (uint8_t)(rdlr >> 16);
  msg->data[3] = (uint8_t)(rdlr >> 24);
  msg->data[4] = (uint8_t)(rdhr);
  msg->data[5] = (uint8_t)(rdhr >> 8);
  msg->data[6] = (uint8_t)(rdhr >> 16);
  msg->data[7] = (uint8_t)(rdhr >> 24);

  /* Release the FIFO output mailbox */
  *rfr |= CAN_RF0R_RFOM0;

  return DRV_OK;
}

void CAN_EnableRxInterrupt(CAN_Handle_t *pHandle, uint8_t fifo) {
  if (fifo == 0) {
    pHandle->pCANx->IER |= CAN_IER_FMPIE0;
  } else {
    pHandle->pCANx->IER |= CAN_IER_FMPIE1;
  }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  IRQ Handlers
 * ═══════════════════════════════════════════════════════════════════════════
 */

void CAN_TX_IRQHandler(CAN_Handle_t *pHandle) {
  CAN_RegDef_t *pCANx = pHandle->pCANx;

  if (pCANx->TSR & CAN_TSR_RQCP0) {
    pCANx->TSR |= CAN_TSR_RQCP0; /* Clear by writing 1 */
    if (pHandle->tx_callback) {
      pHandle->tx_callback(pHandle->tx_cb_context);
    }
  }
}

void CAN_RX0_IRQHandler(CAN_Handle_t *pHandle) {
  CAN_RegDef_t *pCANx = pHandle->pCANx;

  while (pCANx->RF0R & CAN_RF0R_FMP0_MASK) {
    CAN_RxMessage_t msg;
    CAN_Receive(pHandle, 0, &msg, 1);

    if (pHandle->rx_callback) {
      pHandle->rx_callback(pHandle->rx_cb_context, &msg);
    }
  }
}

void CAN_RX1_IRQHandler(CAN_Handle_t *pHandle) {
  CAN_RegDef_t *pCANx = pHandle->pCANx;

  while (pCANx->RF1R & CAN_RF0R_FMP0_MASK) {
    CAN_RxMessage_t msg;
    CAN_Receive(pHandle, 1, &msg, 1);

    if (pHandle->rx_callback) {
      pHandle->rx_callback(pHandle->rx_cb_context, &msg);
    }
  }
}

void CAN_SCE_IRQHandler(CAN_Handle_t *pHandle) {
  uint32_t esr = pHandle->pCANx->ESR;

  if (pHandle->error_callback) {
    pHandle->error_callback(pHandle->error_cb_context, esr);
  }

  /* Clear error flags */
  pHandle->pCANx->MSR |= (1 << 2); /* ERRI */
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Status
 * ═══════════════════════════════════════════════════════════════════════════
 */

uint8_t CAN_GetTxMailboxFreeCount(CAN_Handle_t *pHandle) {
  uint8_t count = 0;
  if (pHandle->pCANx->TSR & CAN_TSR_TME0)
    count++;
  if (pHandle->pCANx->TSR & CAN_TSR_TME1)
    count++;
  if (pHandle->pCANx->TSR & CAN_TSR_TME2)
    count++;
  return count;
}

uint8_t CAN_GetRxFifoLevel(CAN_Handle_t *pHandle, uint8_t fifo) {
  if (fifo == 0) {
    return (uint8_t)(pHandle->pCANx->RF0R & CAN_RF0R_FMP0_MASK);
  } else {
    return (uint8_t)(pHandle->pCANx->RF1R & CAN_RF0R_FMP0_MASK);
  }
}

uint32_t CAN_GetErrorCode(CAN_Handle_t *pHandle) { return pHandle->pCANx->ESR; }

/* ═══════════════════════════════════════════════════════════════════════════
 *  Callbacks
 * ═══════════════════════════════════════════════════════════════════════════
 */

void CAN_RegisterRxCallback(CAN_Handle_t *pHandle, can_rx_callback_t cb,
                            void *ctx) {
  pHandle->rx_callback = cb;
  pHandle->rx_cb_context = ctx;
}

void CAN_RegisterTxCallback(CAN_Handle_t *pHandle, can_tx_callback_t cb,
                            void *ctx) {
  pHandle->tx_callback = cb;
  pHandle->tx_cb_context = ctx;
}

void CAN_RegisterErrorCallback(CAN_Handle_t *pHandle, can_error_callback_t cb,
                               void *ctx) {
  pHandle->error_callback = cb;
  pHandle->error_cb_context = ctx;
}

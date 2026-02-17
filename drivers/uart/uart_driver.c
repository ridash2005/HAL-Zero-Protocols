/**
 * @file    uart_driver.c
 * @brief   STM32F4xx UART/USART Driver — Implementation
 * @version 2.0.0
 *
 * @details Implements all three transfer modes:
 *          - Polling: CPU waits in a loop for TXE/RXNE flags
 *          - Interrupt: ISR fills/drains buffers, signals via callback
 *          - DMA: Configures DMA streams for zero-CPU-overhead transfers
 */

#include "uart_driver.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  Internal Helpers
 * ═══════════════════════════════════════════════════════════════════════════
 */

/**
 * @brief  Calculate and set the BRR (Baud Rate Register) value.
 * @note   BRR = f_PCLK / (8 * (2 - OVER8) * baud_rate)
 *         For OVER16: BRR = f_PCLK / baud_rate, decomposed into
 * mantissa+fraction.
 */
static void UART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t baud_rate) {
  uint32_t pclk;

  /*
   * Determine the peripheral clock:
   *  - USART1, USART6 → APB2 bus
   *  - USART2, USART3, UART4, UART5 → APB1 bus
   * For simplicity, we assume the default HSI (16 MHz) with no prescalers.
   * In production, read the actual RCC CFGR to compute APB1/APB2 clocks.
   */
  if (pUSARTx == USART1 || pUSARTx == USART6) {
    pclk = SYSTEM_CLOCK_HZ; /* APB2 clock */
  } else {
    pclk = SYSTEM_CLOCK_HZ; /* APB1 clock (same with HSI, no prescaler) */
  }

  uint32_t over8 = (pUSARTx->CR1 & USART_CR1_OVER8) ? 1 : 0;
  uint32_t usart_div;

  if (over8) {
    /* OVER8: Mantissa[15:4], Fraction[2:0] (3 bits) */
    usart_div = ((2 * pclk) + (baud_rate / 2)) / baud_rate;
    uint32_t mantissa = usart_div / 16;
    uint32_t fraction = (usart_div % 16) >> 1; /* 3-bit fraction */
    pUSARTx->BRR = (mantissa << 4) | (fraction & 0x07);
  } else {
    /* OVER16: Mantissa[15:4], Fraction[3:0] (4 bits) */
    usart_div = (pclk + (baud_rate / 2)) / baud_rate;
    pUSARTx->BRR = usart_div;
  }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Clock Control
 * ═══════════════════════════════════════════════════════════════════════════
 */

void UART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t en) {
  if (en == ENABLE) {
    if (pUSARTx == USART1) {
      USART1_PCLK_EN();
    } else if (pUSARTx == USART2) {
      USART2_PCLK_EN();
    } else if (pUSARTx == USART3) {
      USART3_PCLK_EN();
    } else if (pUSARTx == UART4) {
      UART4_PCLK_EN();
    } else if (pUSARTx == UART5) {
      UART5_PCLK_EN();
    } else if (pUSARTx == USART6) {
      USART6_PCLK_EN();
    }
  } else {
    if (pUSARTx == USART1) {
      USART1_PCLK_DI();
    } else if (pUSARTx == USART2) {
      USART2_PCLK_DI();
    } else if (pUSARTx == USART3) {
      USART3_PCLK_DI();
    } else if (pUSARTx == UART4) {
      UART4_PCLK_DI();
    } else if (pUSARTx == UART5) {
      UART5_PCLK_DI();
    } else if (pUSARTx == USART6) {
      USART6_PCLK_DI();
    }
  }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Initialization
 * ═══════════════════════════════════════════════════════════════════════════
 */

drv_status_t UART_Init(UART_Handle_t *pHandle) {
  if (pHandle == NULL) {
    return DRV_INVALID_PARAM;
  }

  USART_RegDef_t *pUSARTx = pHandle->pUSARTx;

  /* Enable peripheral clock */
  UART_PeriClockControl(pUSARTx, ENABLE);

  /* ── CR1 Configuration ──────────────────────────────────────────── */
  uint32_t cr1 = 0;

  /* Word length */
  if (pHandle->config.word_length == UART_WORDLEN_9BIT) {
    cr1 |= USART_CR1_M;
  }

  /* Parity */
  if (pHandle->config.parity == UART_PARITY_EVEN) {
    cr1 |= USART_CR1_PCE;
    /* PS bit = 0 → Even (default) */
  } else if (pHandle->config.parity == UART_PARITY_ODD) {
    cr1 |= (USART_CR1_PCE | USART_CR1_PS);
  }

  /* TX/RX enable */
  if (pHandle->config.mode == UART_MODE_TX ||
      pHandle->config.mode == UART_MODE_TXRX) {
    cr1 |= USART_CR1_TE;
  }
  if (pHandle->config.mode == UART_MODE_RX ||
      pHandle->config.mode == UART_MODE_TXRX) {
    cr1 |= USART_CR1_RE;
  }

  /* Oversampling */
  if (pHandle->config.oversampling == UART_OVERSAMPLE_8) {
    cr1 |= USART_CR1_OVER8;
  }

  pUSARTx->CR1 = cr1;

  /* ── CR2 Configuration (Stop bits) ──────────────────────────────── */
  pUSARTx->CR2 &= ~USART_CR2_STOP_MASK;
  pUSARTx->CR2 |= ((uint32_t)pHandle->config.stop_bits << 12);

  /* ── CR3 Configuration (Hardware flow control) ──────────────────── */
  pUSARTx->CR3 = 0;
  if (pHandle->config.hw_flow_control == UART_HW_FLOW_CTS ||
      pHandle->config.hw_flow_control == UART_HW_FLOW_CTS_RTS) {
    pUSARTx->CR3 |= USART_CR3_CTSE;
  }
  if (pHandle->config.hw_flow_control == UART_HW_FLOW_RTS ||
      pHandle->config.hw_flow_control == UART_HW_FLOW_CTS_RTS) {
    pUSARTx->CR3 |= USART_CR3_RTSE;
  }

  /* ── Baud Rate ──────────────────────────────────────────────────── */
  UART_SetBaudRate(pUSARTx, pHandle->config.baud_rate);

  /* Initialize transfer state */
  pHandle->tx_state = XFER_STATE_READY;
  pHandle->rx_state = XFER_STATE_READY;
  pHandle->tx_count = 0;
  pHandle->rx_count = 0;

  return DRV_OK;
}

drv_status_t UART_DeInit(USART_RegDef_t *pUSARTx) {
  if (pUSARTx == USART1) {
    USART1_REG_RESET();
  } else if (pUSARTx == USART2) {
    USART2_REG_RESET();
  } else {
    return DRV_INVALID_PARAM;
  }
  return DRV_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Peripheral Control & Utility
 * ═══════════════════════════════════════════════════════════════════════════
 */

void UART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t en) {
  if (en == ENABLE) {
    pUSARTx->CR1 |= USART_CR1_UE;
  } else {
    pUSARTx->CR1 &= ~USART_CR1_UE;
  }
}

uint8_t UART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t flag) {
  return (pUSARTx->SR & flag) ? FLAG_SET : FLAG_RESET;
}

void UART_ClearFlag(USART_RegDef_t *pUSARTx, uint32_t flag) {
  pUSARTx->SR &= ~flag;
}

void UART_RegisterCallback(UART_Handle_t *pHandle, uart_callback_t cb,
                           void *ctx) {
  pHandle->callback = cb;
  pHandle->cb_context = ctx;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Polling (Blocking) Transfer
 * ═══════════════════════════════════════════════════════════════════════════
 */

drv_status_t UART_Transmit(UART_Handle_t *pHandle, uint8_t *pData,
                           uint32_t len) {
  if (pHandle == NULL || pData == NULL || len == 0) {
    return DRV_INVALID_PARAM;
  }

  USART_RegDef_t *pUSARTx = pHandle->pUSARTx;

  /* Enable the USART peripheral */
  UART_PeripheralControl(pUSARTx, ENABLE);

  for (uint32_t i = 0; i < len; i++) {
    /* Wait until TXE (Transmit Data Register Empty) is set */
    while (!(pUSARTx->SR & USART_SR_TXE))
      ;

    /* Write data to the data register */
    if (pHandle->config.word_length == UART_WORDLEN_9BIT) {
      /* 9-bit mode: take 2 bytes per frame */
      uint16_t data9 = *((uint16_t *)pData) & 0x01FF;
      pUSARTx->DR = data9;
      if (pHandle->config.parity == UART_PARITY_NONE) {
        pData += 2; /* 9 data bits, no parity → 2 bytes consumed */
        i++;        /* consumed extra byte */
      } else {
        pData++;
      }
    } else {
      /* 8-bit mode */
      pUSARTx->DR = (*pData & 0xFF);
      pData++;
    }
  }

  /* Wait until TC (Transmission Complete) is set */
  while (!(pUSARTx->SR & USART_SR_TC))
    ;

  return DRV_OK;
}

drv_status_t UART_Receive(UART_Handle_t *pHandle, uint8_t *pData,
                          uint32_t len) {
  if (pHandle == NULL || pData == NULL || len == 0) {
    return DRV_INVALID_PARAM;
  }

  USART_RegDef_t *pUSARTx = pHandle->pUSARTx;

  /* Enable the USART peripheral */
  UART_PeripheralControl(pUSARTx, ENABLE);

  for (uint32_t i = 0; i < len; i++) {
    /* Wait until RXNE (Read Data Register Not Empty) is set */
    while (!(pUSARTx->SR & USART_SR_RXNE))
      ;

    /* Read data from the data register */
    if (pHandle->config.word_length == UART_WORDLEN_9BIT) {
      if (pHandle->config.parity == UART_PARITY_NONE) {
        *((uint16_t *)pData) = (uint16_t)(pUSARTx->DR & 0x01FF);
        pData += 2;
        i++;
      } else {
        /* Parity used: 8 data bits + 1 parity bit */
        *pData = (uint8_t)(pUSARTx->DR & 0xFF);
        pData++;
      }
    } else {
      if (pHandle->config.parity == UART_PARITY_NONE) {
        *pData = (uint8_t)(pUSARTx->DR & 0xFF);
      } else {
        /* Parity used: 7 data bits + 1 parity bit */
        *pData = (uint8_t)(pUSARTx->DR & 0x7F);
      }
      pData++;
    }
  }

  return DRV_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Interrupt (Non-Blocking) Transfer
 * ═══════════════════════════════════════════════════════════════════════════
 */

drv_status_t UART_Transmit_IT(UART_Handle_t *pHandle, uint8_t *pData,
                              uint32_t len) {
  if (pHandle == NULL || pData == NULL || len == 0) {
    return DRV_INVALID_PARAM;
  }

  if (pHandle->tx_state == XFER_STATE_BUSY_TX) {
    return DRV_BUSY;
  }

  /* Store transfer parameters */
  pHandle->pTxBuffer = pData;
  pHandle->tx_len = len;
  pHandle->tx_count = 0;
  pHandle->tx_state = XFER_STATE_BUSY_TX;

  /* Enable the USART peripheral */
  UART_PeripheralControl(pHandle->pUSARTx, ENABLE);

  /* Enable TXE interrupt → ISR will start transmitting */
  pHandle->pUSARTx->CR1 |= USART_CR1_TXEIE;

  /* Enable TC interrupt for transfer-complete notification */
  pHandle->pUSARTx->CR1 |= USART_CR1_TCIE;

  return DRV_OK;
}

drv_status_t UART_Receive_IT(UART_Handle_t *pHandle, uint8_t *pData,
                             uint32_t len) {
  if (pHandle == NULL || pData == NULL || len == 0) {
    return DRV_INVALID_PARAM;
  }

  if (pHandle->rx_state == XFER_STATE_BUSY_RX) {
    return DRV_BUSY;
  }

  /* Store transfer parameters */
  pHandle->pRxBuffer = pData;
  pHandle->rx_len = len;
  pHandle->rx_count = 0;
  pHandle->rx_state = XFER_STATE_BUSY_RX;

  /* Enable the USART peripheral */
  UART_PeripheralControl(pHandle->pUSARTx, ENABLE);

  /* Enable RXNE interrupt */
  pHandle->pUSARTx->CR1 |= USART_CR1_RXNEIE;

  /* Enable error interrupts */
  pHandle->pUSARTx->CR3 |= USART_CR3_EIE;

  return DRV_OK;
}

/**
 * @brief  UART IRQ Handler — call this from USARTx_IRQHandler
 *
 * Handles TXE, TC, RXNE, and error flags.
 */
void UART_IRQHandler(UART_Handle_t *pHandle) {
  USART_RegDef_t *pUSARTx = pHandle->pUSARTx;
  uint32_t sr = pUSARTx->SR;
  uint32_t cr1 = pUSARTx->CR1;
  uint32_t cr3 = pUSARTx->CR3;

  /* ── TXE Interrupt (Transmit Data Register Empty) ────────────── */
  if ((sr & USART_SR_TXE) && (cr1 & USART_CR1_TXEIE)) {
    if (pHandle->tx_state == XFER_STATE_BUSY_TX) {
      if (pHandle->tx_count < pHandle->tx_len) {
        pUSARTx->DR = pHandle->pTxBuffer[pHandle->tx_count++];
      }
      if (pHandle->tx_count >= pHandle->tx_len) {
        /* All bytes loaded → disable TXE interrupt, wait for TC */
        pUSARTx->CR1 &= ~USART_CR1_TXEIE;
      }
    }
  }

  /* ── TC Interrupt (Transmission Complete) ────────────────────── */
  if ((sr & USART_SR_TC) && (cr1 & USART_CR1_TCIE)) {
    if (pHandle->tx_state == XFER_STATE_BUSY_TX &&
        pHandle->tx_count >= pHandle->tx_len) {
      /* Transfer finished */
      pUSARTx->CR1 &= ~USART_CR1_TCIE;
      pUSARTx->SR &= ~USART_SR_TC;
      pHandle->tx_state = XFER_STATE_READY;

      if (pHandle->callback) {
        pHandle->callback(pHandle->cb_context, EVT_TX_COMPLETE);
      }
    }
  }

  /* ── RXNE Interrupt (Receive Data Register Not Empty) ────────── */
  if ((sr & USART_SR_RXNE) && (cr1 & USART_CR1_RXNEIE)) {
    if (pHandle->rx_state == XFER_STATE_BUSY_RX) {
      if (pHandle->rx_count < pHandle->rx_len) {
        if (pHandle->config.parity == UART_PARITY_NONE) {
          pHandle->pRxBuffer[pHandle->rx_count++] =
              (uint8_t)(pUSARTx->DR & 0xFF);
        } else {
          pHandle->pRxBuffer[pHandle->rx_count++] =
              (uint8_t)(pUSARTx->DR & 0x7F);
        }
      }
      if (pHandle->rx_count >= pHandle->rx_len) {
        /* All bytes received */
        pUSARTx->CR1 &= ~USART_CR1_RXNEIE;
        pHandle->rx_state = XFER_STATE_READY;

        if (pHandle->callback) {
          pHandle->callback(pHandle->cb_context, EVT_RX_COMPLETE);
        }
      }
    }
  }

  /* ── Error Handling ──────────────────────────────────────────── */
  if (cr3 & USART_CR3_EIE) {
    if (sr & USART_SR_ORE) {
      /* Overrun Error: clear by reading SR then DR */
      (void)pUSARTx->SR;
      (void)pUSARTx->DR;
      if (pHandle->callback) {
        pHandle->callback(pHandle->cb_context, EVT_ERROR);
      }
    }
    if (sr & USART_SR_FE) {
      (void)pUSARTx->SR;
      (void)pUSARTx->DR;
    }
    if (sr & USART_SR_NF) {
      (void)pUSARTx->SR;
      (void)pUSARTx->DR;
    }
  }

  /* ── IDLE Line Detection ─────────────────────────────────────── */
  if ((sr & USART_SR_IDLE) && (cr1 & USART_CR1_IDLEIE)) {
    /* Clear IDLE flag by reading SR then DR */
    (void)pUSARTx->SR;
    (void)pUSARTx->DR;
    if (pHandle->callback) {
      pHandle->callback(pHandle->cb_context, EVT_IDLE);
    }
  }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  DMA Transfer
 * ═══════════════════════════════════════════════════════════════════════════
 */

drv_status_t UART_Transmit_DMA(UART_Handle_t *pHandle, uint8_t *pData,
                               uint32_t len) {
  if (pHandle == NULL || pData == NULL || len == 0) {
    return DRV_INVALID_PARAM;
  }

  if (pHandle->tx_state == XFER_STATE_BUSY_TX) {
    return DRV_BUSY;
  }

  pHandle->tx_state = XFER_STATE_BUSY_TX;

  /*
   * DMA Configuration for UART TX:
   *
   * The user must configure the appropriate DMA stream/channel before calling
   * this function. The DMA driver (dma_driver.h) provides the API.
   *
   * Typical mapping (STM32F4):
   *   USART2_TX → DMA1 Stream 6, Channel 4
   *   USART1_TX → DMA2 Stream 7, Channel 4
   *
   * Steps:
   * 1. Configure DMA stream (direction, addresses, data count)
   * 2. Enable DMAT bit in USART CR3
   * 3. Enable the DMA stream
   * 4. The DMA will service TXE requests automatically
   * 5. On DMA TC interrupt, clear DMAT and notify application
   */

  /* Enable DMA transmitter in USART */
  pHandle->pUSARTx->CR3 |= USART_CR3_DMAT;

  /* Enable USART peripheral */
  UART_PeripheralControl(pHandle->pUSARTx, ENABLE);

  /*
   * NOTE: The actual DMA stream configuration (PAR, M0AR, NDTR, CR)
   * should be done via the DMA driver before calling this function.
   * This function merely enables the USART-side DMA request.
   */

  return DRV_OK;
}

drv_status_t UART_Receive_DMA(UART_Handle_t *pHandle, uint8_t *pData,
                              uint32_t len) {
  if (pHandle == NULL || pData == NULL || len == 0) {
    return DRV_INVALID_PARAM;
  }

  if (pHandle->rx_state == XFER_STATE_BUSY_RX) {
    return DRV_BUSY;
  }

  pHandle->rx_state = XFER_STATE_BUSY_RX;

  /* Enable DMA receiver in USART */
  pHandle->pUSARTx->CR3 |= USART_CR3_DMAR;

  /* Enable IDLE Line interrupt for variable-length DMA reception */
  pHandle->pUSARTx->CR1 |= USART_CR1_IDLEIE;

  /* Enable USART peripheral */
  UART_PeripheralControl(pHandle->pUSARTx, ENABLE);

  return DRV_OK;
}

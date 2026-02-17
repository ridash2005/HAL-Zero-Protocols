/**
 * @file    spi_driver.c
 * @brief   STM32F4xx SPI Driver — Implementation
 * @version 2.0.0
 */

#include "spi_driver.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  Clock Control
 * ═══════════════════════════════════════════════════════════════════════════
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t en) {
  if (en == ENABLE) {
    if (pSPIx == SPI1) {
      SPI1_PCLK_EN();
    } else if (pSPIx == SPI2) {
      SPI2_PCLK_EN();
    } else if (pSPIx == SPI3) {
      SPI3_PCLK_EN();
    } else if (pSPIx == SPI4) {
      SPI4_PCLK_EN();
    }
  } else {
    if (pSPIx == SPI1) {
      SPI1_PCLK_DI();
    } else if (pSPIx == SPI2) {
      SPI2_PCLK_DI();
    } else if (pSPIx == SPI3) {
      SPI3_PCLK_DI();
    } else if (pSPIx == SPI4) {
      SPI4_PCLK_DI();
    }
  }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Initialization
 * ═══════════════════════════════════════════════════════════════════════════
 */

drv_status_t SPI_Init(SPI_Handle_t *pHandle) {
  if (pHandle == NULL) {
    return DRV_INVALID_PARAM;
  }

  SPI_RegDef_t *pSPIx = pHandle->pSPIx;
  SPI_Config_t *cfg = &pHandle->config;

  /* Enable peripheral clock */
  SPI_PeriClockControl(pSPIx, ENABLE);

  /* Build CR1 configuration */
  uint32_t cr1 = 0;

  /* Device mode (Master/Slave) */
  if (cfg->device_mode == SPI_DEVICE_MODE_MASTER) {
    cr1 |= SPI_CR1_MSTR;
  }

  /* Bus configuration */
  if (cfg->bus_config == SPI_BUS_FULLDUPLEX) {
    /* BIDIMODE=0, RXONLY=0 → full-duplex */
    cr1 &= ~SPI_CR1_BIDIMODE;
  } else if (cfg->bus_config == SPI_BUS_HALFDUPLEX) {
    /* BIDIMODE=1 → half-duplex */
    cr1 |= SPI_CR1_BIDIMODE;
  } else if (cfg->bus_config == SPI_BUS_SIMPLEX_RXONLY) {
    /* BIDIMODE=0, RXONLY=1 → simplex receive */
    cr1 &= ~SPI_CR1_BIDIMODE;
    cr1 |= SPI_CR1_RXONLY;
  }

  /* Clock speed (baud rate prescaler) */
  cr1 |= ((uint32_t)cfg->sclk_speed << 3);

  /* Data frame format (8-bit or 16-bit) */
  if (cfg->dff == SPI_DFF_16BIT) {
    cr1 |= SPI_CR1_DFF;
  }

  /* Clock polarity */
  if (cfg->cpol == SPI_CPOL_HIGH) {
    cr1 |= SPI_CR1_CPOL;
  }

  /* Clock phase */
  if (cfg->cpha == SPI_CPHA_2EDGE) {
    cr1 |= SPI_CR1_CPHA;
  }

  /* Software slave management */
  if (cfg->ssm == SPI_SSM_ENABLE) {
    cr1 |= SPI_CR1_SSM;
    /* Set SSI high to avoid MODF error in master mode */
    if (cfg->device_mode == SPI_DEVICE_MODE_MASTER) {
      cr1 |= SPI_CR1_SSI;
    }
  }

  pSPIx->CR1 = cr1;

  /* Initialize transfer state */
  pHandle->tx_state = XFER_STATE_READY;
  pHandle->rx_state = XFER_STATE_READY;

  return DRV_OK;
}

drv_status_t SPI_DeInit(SPI_RegDef_t *pSPIx) {
  if (pSPIx == SPI1) {
    SPI1_REG_RESET();
  } else if (pSPIx == SPI2) {
    SPI2_REG_RESET();
  } else if (pSPIx == SPI3) {
    SPI3_REG_RESET();
  } else {
    return DRV_INVALID_PARAM;
  }
  return DRV_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Control Functions
 * ═══════════════════════════════════════════════════════════════════════════
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t en) {
  if (en == ENABLE) {
    pSPIx->CR1 |= SPI_CR1_SPE;
  } else {
    /* Wait until BSY flag is cleared before disabling */
    while (pSPIx->SR & SPI_SR_BSY)
      ;
    pSPIx->CR1 &= ~SPI_CR1_SPE;
  }
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t en) {
  if (en == ENABLE) {
    pSPIx->CR1 |= SPI_CR1_SSI;
  } else {
    pSPIx->CR1 &= ~SPI_CR1_SSI;
  }
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t en) {
  if (en == ENABLE) {
    pSPIx->CR2 |= SPI_CR2_SSOE;
  } else {
    pSPIx->CR2 &= ~SPI_CR2_SSOE;
  }
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flag) {
  return (pSPIx->SR & flag) ? FLAG_SET : FLAG_RESET;
}

void SPI_RegisterCallback(SPI_Handle_t *pHandle, spi_callback_t cb, void *ctx) {
  pHandle->callback = cb;
  pHandle->cb_context = ctx;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Polling (Blocking) Transfers
 * ═══════════════════════════════════════════════════════════════════════════
 */

drv_status_t SPI_Transmit(SPI_Handle_t *pHandle, uint8_t *pData, uint32_t len) {
  if (pHandle == NULL || pData == NULL || len == 0) {
    return DRV_INVALID_PARAM;
  }

  SPI_RegDef_t *pSPIx = pHandle->pSPIx;

  while (len > 0) {
    /* Wait until TXE flag is set */
    while (!(pSPIx->SR & SPI_SR_TXE))
      ;

    if (pHandle->config.dff == SPI_DFF_16BIT) {
      /* 16-bit data frame */
      pSPIx->DR = *((uint16_t *)pData);
      pData += 2;
      len -= 2;
    } else {
      /* 8-bit data frame */
      pSPIx->DR = *pData;
      pData++;
      len--;
    }
  }

  /* Wait for BSY flag to clear (transmission complete) */
  while (pSPIx->SR & SPI_SR_BSY)
    ;

  /* Clear OVR flag by reading DR then SR */
  (void)pSPIx->DR;
  (void)pSPIx->SR;

  return DRV_OK;
}

drv_status_t SPI_Receive(SPI_Handle_t *pHandle, uint8_t *pData, uint32_t len) {
  if (pHandle == NULL || pData == NULL || len == 0) {
    return DRV_INVALID_PARAM;
  }

  SPI_RegDef_t *pSPIx = pHandle->pSPIx;

  while (len > 0) {
    /* Send dummy byte to generate clock (if master) */
    while (!(pSPIx->SR & SPI_SR_TXE))
      ;
    pSPIx->DR = 0xFF;

    /* Wait for RXNE */
    while (!(pSPIx->SR & SPI_SR_RXNE))
      ;

    if (pHandle->config.dff == SPI_DFF_16BIT) {
      *((uint16_t *)pData) = (uint16_t)pSPIx->DR;
      pData += 2;
      len -= 2;
    } else {
      *pData = (uint8_t)pSPIx->DR;
      pData++;
      len--;
    }
  }

  return DRV_OK;
}

drv_status_t SPI_TransmitReceive(SPI_Handle_t *pHandle, uint8_t *pTxData,
                                 uint8_t *pRxData, uint32_t len) {
  if (pHandle == NULL || pTxData == NULL || pRxData == NULL || len == 0) {
    return DRV_INVALID_PARAM;
  }

  SPI_RegDef_t *pSPIx = pHandle->pSPIx;

  while (len > 0) {
    /* Wait for TXE and send data */
    while (!(pSPIx->SR & SPI_SR_TXE))
      ;

    if (pHandle->config.dff == SPI_DFF_16BIT) {
      pSPIx->DR = *((uint16_t *)pTxData);
      pTxData += 2;
    } else {
      pSPIx->DR = *pTxData;
      pTxData++;
    }

    /* Wait for RXNE and read data */
    while (!(pSPIx->SR & SPI_SR_RXNE))
      ;

    if (pHandle->config.dff == SPI_DFF_16BIT) {
      *((uint16_t *)pRxData) = (uint16_t)pSPIx->DR;
      pRxData += 2;
      len -= 2;
    } else {
      *pRxData = (uint8_t)pSPIx->DR;
      pRxData++;
      len--;
    }
  }

  while (pSPIx->SR & SPI_SR_BSY)
    ;

  return DRV_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Interrupt (Non-Blocking) Transfers
 * ═══════════════════════════════════════════════════════════════════════════
 */

drv_status_t SPI_Transmit_IT(SPI_Handle_t *pHandle, uint8_t *pData,
                             uint32_t len) {
  if (pHandle == NULL || pData == NULL || len == 0) {
    return DRV_INVALID_PARAM;
  }

  if (pHandle->tx_state == XFER_STATE_BUSY_TX) {
    return DRV_BUSY;
  }

  pHandle->pTxBuffer = pData;
  pHandle->tx_len = len;
  pHandle->tx_state = XFER_STATE_BUSY_TX;

  /* Enable TXEIE to trigger interrupt when TXE flag is set */
  pHandle->pSPIx->CR2 |= SPI_CR2_TXEIE;

  return DRV_OK;
}

drv_status_t SPI_Receive_IT(SPI_Handle_t *pHandle, uint8_t *pData,
                            uint32_t len) {
  if (pHandle == NULL || pData == NULL || len == 0) {
    return DRV_INVALID_PARAM;
  }

  if (pHandle->rx_state == XFER_STATE_BUSY_RX) {
    return DRV_BUSY;
  }

  pHandle->pRxBuffer = pData;
  pHandle->rx_len = len;
  pHandle->rx_state = XFER_STATE_BUSY_RX;

  /* Enable RXNEIE to trigger interrupt when RXNE flag is set */
  pHandle->pSPIx->CR2 |= SPI_CR2_RXNEIE;

  return DRV_OK;
}

void SPI_IRQHandler(SPI_Handle_t *pHandle) {
  SPI_RegDef_t *pSPIx = pHandle->pSPIx;
  uint32_t sr = pSPIx->SR;
  uint32_t cr2 = pSPIx->CR2;

  /* ── TXE Interrupt ───────────────────────────────────────────── */
  if ((sr & SPI_SR_TXE) && (cr2 & SPI_CR2_TXEIE)) {
    if (pHandle->tx_len > 0) {
      if (pHandle->config.dff == SPI_DFF_16BIT) {
        pSPIx->DR = *((uint16_t *)pHandle->pTxBuffer);
        pHandle->pTxBuffer += 2;
        pHandle->tx_len -= 2;
      } else {
        pSPIx->DR = *pHandle->pTxBuffer;
        pHandle->pTxBuffer++;
        pHandle->tx_len--;
      }
    }
    if (pHandle->tx_len == 0) {
      /* Close the SPI transmission */
      pSPIx->CR2 &= ~SPI_CR2_TXEIE;
      pHandle->pTxBuffer = NULL;
      pHandle->tx_state = XFER_STATE_READY;

      if (pHandle->callback) {
        pHandle->callback(pHandle->cb_context, EVT_TX_COMPLETE);
      }
    }
  }

  /* ── RXNE Interrupt ──────────────────────────────────────────── */
  if ((sr & SPI_SR_RXNE) && (cr2 & SPI_CR2_RXNEIE)) {
    if (pHandle->rx_len > 0) {
      if (pHandle->config.dff == SPI_DFF_16BIT) {
        *((uint16_t *)pHandle->pRxBuffer) = (uint16_t)pSPIx->DR;
        pHandle->pRxBuffer += 2;
        pHandle->rx_len -= 2;
      } else {
        *pHandle->pRxBuffer = (uint8_t)pSPIx->DR;
        pHandle->pRxBuffer++;
        pHandle->rx_len--;
      }
    }
    if (pHandle->rx_len == 0) {
      /* Close the SPI reception */
      pSPIx->CR2 &= ~SPI_CR2_RXNEIE;
      pHandle->pRxBuffer = NULL;
      pHandle->rx_state = XFER_STATE_READY;

      if (pHandle->callback) {
        pHandle->callback(pHandle->cb_context, EVT_RX_COMPLETE);
      }
    }
  }

  /* ── OVR Error ───────────────────────────────────────────────── */
  if ((sr & SPI_SR_OVR) && (cr2 & SPI_CR2_ERRIE)) {
    /* Clear OVR by reading DR then SR */
    (void)pSPIx->DR;
    (void)pSPIx->SR;

    if (pHandle->callback) {
      pHandle->callback(pHandle->cb_context, EVT_ERROR);
    }
  }
}

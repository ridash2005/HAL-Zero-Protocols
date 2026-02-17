/**
 * @file    i2c_driver.c
 * @brief   STM32F4xx I2C Driver — Implementation
 * @version 2.0.0
 */

#include "i2c_driver.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  Internal Helpers
 * ═══════════════════════════════════════════════════════════════════════════
 */

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx) {
  pI2Cx->CR1 |= I2C_CR1_START;
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx) {
  pI2Cx->CR1 |= I2C_CR1_STOP;
}

/**
 * @brief Send the slave address with R/W bit.
 * @param rw: 0 = Write, 1 = Read
 */
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t slave_addr,
                                    uint8_t rw) {
  slave_addr = slave_addr << 1;
  if (rw) {
    slave_addr |= 1; /* Set read bit */
  } else {
    slave_addr &= ~1; /* Clear read bit */
  }
  pI2Cx->DR = slave_addr;
}

/**
 * @brief Clear the ADDR flag by reading SR1 followed by SR2.
 */
static void I2C_ClearADDRFlag(I2C_Handle_t *pHandle) {
  uint32_t dummy;

  /* Check device mode */
  if (pHandle->pI2Cx->SR2 & I2C_SR2_MSL) {
    /* Master mode */
    if (pHandle->state == XFER_STATE_BUSY_RX) {
      if (pHandle->tx_rx_size == 1) {
        /* Disable ACK before clearing ADDR (single byte reception) */
        I2C_ManageACK(pHandle->pI2Cx, DISABLE);
      }
    }
  }

  /* Clear ADDR flag */
  dummy = pHandle->pI2Cx->SR1;
  dummy = pHandle->pI2Cx->SR2;
  (void)dummy;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Clock Control
 * ═══════════════════════════════════════════════════════════════════════════
 */

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t en) {
  if (en == ENABLE) {
    if (pI2Cx == I2C1) {
      I2C1_PCLK_EN();
    } else if (pI2Cx == I2C2) {
      I2C2_PCLK_EN();
    } else if (pI2Cx == I2C3) {
      I2C3_PCLK_EN();
    }
  } else {
    if (pI2Cx == I2C1) {
      I2C1_PCLK_DI();
    } else if (pI2Cx == I2C2) {
      I2C2_PCLK_DI();
    } else if (pI2Cx == I2C3) {
      I2C3_PCLK_DI();
    }
  }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Initialization
 * ═══════════════════════════════════════════════════════════════════════════
 */

drv_status_t I2C_Init(I2C_Handle_t *pHandle) {
  if (pHandle == NULL) {
    return DRV_INVALID_PARAM;
  }

  I2C_RegDef_t *pI2Cx = pHandle->pI2Cx;

  /* Enable peripheral clock */
  I2C_PeriClockControl(pI2Cx, ENABLE);

  /* 1. Configure CR2 — peripheral clock frequency */
  uint32_t pclk_mhz = SYSTEM_CLOCK_HZ / 1000000UL;
  pI2Cx->CR2 =
      (pI2Cx->CR2 & ~I2C_CR2_FREQ_MASK) | (pclk_mhz & I2C_CR2_FREQ_MASK);

  /* 2. Configure own address (OAR1) */
  pI2Cx->OAR1 = (pHandle->config.device_address << 1);
  pI2Cx->OAR1 |= (1 << 14); /* Bit 14 must be kept at 1 by software */

  /* 3. Configure CCR (Clock Control Register) */
  uint32_t ccr_val = 0;

  if (pHandle->config.scl_speed <= I2C_SCL_SPEED_SM) {
    /* Standard mode: Thigh = Tlow = CCR × TPCLK1
     * CCR = f_PCLK1 / (2 × f_SCL)  */
    ccr_val = SYSTEM_CLOCK_HZ / (2 * pHandle->config.scl_speed);
  } else {
    /* Fast mode */
    ccr_val |= I2C_CCR_FS;

    if (pHandle->config.fm_duty_cycle == I2C_FM_DUTY_2) {
      /* Duty 0: Tlow/Thigh = 2, CCR = f_PCLK1 / (3 × f_SCL) */
      ccr_val |= (SYSTEM_CLOCK_HZ / (3 * pHandle->config.scl_speed)) & 0xFFF;
    } else {
      /* Duty 1: Tlow/Thigh = 16/9, CCR = f_PCLK1 / (25 × f_SCL) */
      ccr_val |= I2C_CCR_DUTY;
      ccr_val |= (SYSTEM_CLOCK_HZ / (25 * pHandle->config.scl_speed)) & 0xFFF;
    }
  }

  if ((ccr_val & 0xFFF) == 0) {
    ccr_val |= 1; /* CCR minimum value is 1 */
  }
  pI2Cx->CCR = ccr_val;

  /* 4. Configure TRISE register */
  if (pHandle->config.scl_speed <= I2C_SCL_SPEED_SM) {
    /* Standard mode: TRISE = (f_PCLK1 / 1MHz) + 1 */
    pI2Cx->TRISE = pclk_mhz + 1;
  } else {
    /* Fast mode: TRISE = (f_PCLK1 × 300ns / 1e9) + 1 */
    pI2Cx->TRISE = ((pclk_mhz * 300) / 1000) + 1;
  }

  /* 5. Enable ACK */
  if (pHandle->config.ack_control == I2C_ACK_ENABLE) {
    pI2Cx->CR1 |= I2C_CR1_ACK;
  }

  /* Initialize state */
  pHandle->state = XFER_STATE_READY;

  return DRV_OK;
}

drv_status_t I2C_DeInit(I2C_RegDef_t *pI2Cx) {
  if (pI2Cx == I2C1) {
    I2C1_REG_RESET();
  } else if (pI2Cx == I2C2) {
    I2C2_REG_RESET();
  } else if (pI2Cx == I2C3) {
    I2C3_REG_RESET();
  } else {
    return DRV_INVALID_PARAM;
  }
  return DRV_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Control
 * ═══════════════════════════════════════════════════════════════════════════
 */

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t en) {
  if (en == ENABLE) {
    pI2Cx->CR1 |= I2C_CR1_PE;
  } else {
    pI2Cx->CR1 &= ~I2C_CR1_PE;
  }
}

void I2C_ManageACK(I2C_RegDef_t *pI2Cx, uint8_t en) {
  if (en == ENABLE) {
    pI2Cx->CR1 |= I2C_CR1_ACK;
  } else {
    pI2Cx->CR1 &= ~I2C_CR1_ACK;
  }
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flag) {
  return (pI2Cx->SR1 & flag) ? FLAG_SET : FLAG_RESET;
}

void I2C_RegisterCallback(I2C_Handle_t *pHandle, i2c_callback_t cb, void *ctx) {
  pHandle->callback = cb;
  pHandle->cb_context = ctx;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Polling (Blocking) — Master Mode
 * ═══════════════════════════════════════════════════════════════════════════
 */

drv_status_t I2C_MasterTransmit(I2C_Handle_t *pHandle, uint8_t slave_addr,
                                uint8_t *pData, uint32_t len, i2c_rs_t rs) {
  if (pHandle == NULL || pData == NULL) {
    return DRV_INVALID_PARAM;
  }

  I2C_RegDef_t *pI2Cx = pHandle->pI2Cx;

  /* 1. Generate START condition */
  I2C_GenerateStartCondition(pI2Cx);

  /* 2. Wait for SB (Start Bit) flag — confirms START generated */
  while (!I2C_GetFlagStatus(pI2Cx, I2C_SR1_SB))
    ;

  /* 3. Send slave address with Write bit */
  I2C_ExecuteAddressPhase(pI2Cx, slave_addr, 0);

  /* 4. Wait for ADDR flag — address acknowledged */
  while (!I2C_GetFlagStatus(pI2Cx, I2C_SR1_ADDR))
    ;

  /* 5. Clear ADDR flag */
  I2C_ClearADDRFlag(pHandle);

  /* 6. Send data bytes */
  while (len > 0) {
    while (!I2C_GetFlagStatus(pI2Cx, I2C_SR1_TXE))
      ; /* Wait for TXE */
    pI2Cx->DR = *pData;
    pData++;
    len--;
  }

  /* 7. Wait for TXE and BTF — all data shifted out */
  while (!I2C_GetFlagStatus(pI2Cx, I2C_SR1_TXE))
    ;
  while (!I2C_GetFlagStatus(pI2Cx, I2C_SR1_BTF))
    ;

  /* 8. Generate STOP condition (unless repeated START requested) */
  if (rs == I2C_REPEATED_START_DISABLE) {
    I2C_GenerateStopCondition(pI2Cx);
  }

  return DRV_OK;
}

drv_status_t I2C_MasterReceive(I2C_Handle_t *pHandle, uint8_t slave_addr,
                               uint8_t *pData, uint32_t len, i2c_rs_t rs) {
  if (pHandle == NULL || pData == NULL || len == 0) {
    return DRV_INVALID_PARAM;
  }

  I2C_RegDef_t *pI2Cx = pHandle->pI2Cx;

  /* 1. Generate START */
  I2C_GenerateStartCondition(pI2Cx);
  while (!I2C_GetFlagStatus(pI2Cx, I2C_SR1_SB))
    ;

  /* 2. Send address with Read bit */
  I2C_ExecuteAddressPhase(pI2Cx, slave_addr, 1);
  while (!I2C_GetFlagStatus(pI2Cx, I2C_SR1_ADDR))
    ;

  if (len == 1) {
    /* Single byte reception */
    I2C_ManageACK(pI2Cx, DISABLE);
    I2C_ClearADDRFlag(pHandle);

    /* Wait for RXNE */
    while (!I2C_GetFlagStatus(pI2Cx, I2C_SR1_RXNE))
      ;

    if (rs == I2C_REPEATED_START_DISABLE) {
      I2C_GenerateStopCondition(pI2Cx);
    }

    *pData = (uint8_t)pI2Cx->DR;
  } else {
    /* Multi-byte reception */
    I2C_ClearADDRFlag(pHandle);

    for (uint32_t i = len; i > 0; i--) {
      while (!I2C_GetFlagStatus(pI2Cx, I2C_SR1_RXNE))
        ;

      if (i == 2) {
        /* Disable ACK before reading second-to-last byte */
        I2C_ManageACK(pI2Cx, DISABLE);

        if (rs == I2C_REPEATED_START_DISABLE) {
          I2C_GenerateStopCondition(pI2Cx);
        }
      }

      *pData = (uint8_t)pI2Cx->DR;
      pData++;
    }
  }

  /* Re-enable ACK for next transaction */
  if (pHandle->config.ack_control == I2C_ACK_ENABLE) {
    I2C_ManageACK(pI2Cx, ENABLE);
  }

  return DRV_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Interrupt (Non-Blocking) — Master Mode
 * ═══════════════════════════════════════════════════════════════════════════
 */

drv_status_t I2C_MasterTransmit_IT(I2C_Handle_t *pHandle, uint8_t slave_addr,
                                   uint8_t *pData, uint32_t len, i2c_rs_t rs) {
  if (pHandle->state != XFER_STATE_READY) {
    return DRV_BUSY;
  }

  pHandle->pTxBuffer = pData;
  pHandle->tx_len = len;
  pHandle->state = XFER_STATE_BUSY_TX;
  pHandle->slave_addr = slave_addr;
  pHandle->repeated_start = rs;

  /* Generate START — the ISR flow will handle the rest */
  I2C_GenerateStartCondition(pHandle->pI2Cx);

  /* Enable interrupts */
  pHandle->pI2Cx->CR2 |= I2C_CR2_ITBUFEN; /* Buffer interrupt */
  pHandle->pI2Cx->CR2 |= I2C_CR2_ITEVTEN; /* Event interrupt  */
  pHandle->pI2Cx->CR2 |= I2C_CR2_ITERREN; /* Error interrupt  */

  return DRV_OK;
}

drv_status_t I2C_MasterReceive_IT(I2C_Handle_t *pHandle, uint8_t slave_addr,
                                  uint8_t *pData, uint32_t len, i2c_rs_t rs) {
  if (pHandle->state != XFER_STATE_READY) {
    return DRV_BUSY;
  }

  pHandle->pRxBuffer = pData;
  pHandle->rx_len = len;
  pHandle->tx_rx_size = len;
  pHandle->state = XFER_STATE_BUSY_RX;
  pHandle->slave_addr = slave_addr;
  pHandle->repeated_start = rs;

  /* Generate START */
  I2C_GenerateStartCondition(pHandle->pI2Cx);

  /* Enable interrupts */
  pHandle->pI2Cx->CR2 |= I2C_CR2_ITBUFEN;
  pHandle->pI2Cx->CR2 |= I2C_CR2_ITEVTEN;
  pHandle->pI2Cx->CR2 |= I2C_CR2_ITERREN;

  return DRV_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Close Transfer Helpers
 * ═══════════════════════════════════════════════════════════════════════════
 */

void I2C_CloseSendData(I2C_Handle_t *pHandle) {
  /* Disable interrupt bits */
  pHandle->pI2Cx->CR2 &= ~I2C_CR2_ITBUFEN;
  pHandle->pI2Cx->CR2 &= ~I2C_CR2_ITEVTEN;

  pHandle->state = XFER_STATE_READY;
  pHandle->pTxBuffer = NULL;
  pHandle->tx_len = 0;
}

void I2C_CloseReceiveData(I2C_Handle_t *pHandle) {
  pHandle->pI2Cx->CR2 &= ~I2C_CR2_ITBUFEN;
  pHandle->pI2Cx->CR2 &= ~I2C_CR2_ITEVTEN;

  pHandle->state = XFER_STATE_READY;
  pHandle->pRxBuffer = NULL;
  pHandle->rx_len = 0;
  pHandle->tx_rx_size = 0;

  if (pHandle->config.ack_control == I2C_ACK_ENABLE) {
    I2C_ManageACK(pHandle->pI2Cx, ENABLE);
  }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Event IRQ Handler
 * ═══════════════════════════════════════════════════════════════════════════
 */

void I2C_EV_IRQHandler(I2C_Handle_t *pHandle) {
  I2C_RegDef_t *pI2Cx = pHandle->pI2Cx;
  uint32_t sr1 = pI2Cx->SR1;
  uint32_t cr2 = pI2Cx->CR2;

  uint8_t evt_en = (cr2 & I2C_CR2_ITEVTEN) ? 1 : 0;
  uint8_t buf_en = (cr2 & I2C_CR2_ITBUFEN) ? 1 : 0;

  /* ── SB (Start Bit) Event ────────────────────────────────────── */
  if (evt_en && (sr1 & I2C_SR1_SB)) {
    /* START condition generated. Now send address. */
    if (pHandle->state == XFER_STATE_BUSY_TX) {
      I2C_ExecuteAddressPhase(pI2Cx, pHandle->slave_addr, 0);
    } else if (pHandle->state == XFER_STATE_BUSY_RX) {
      I2C_ExecuteAddressPhase(pI2Cx, pHandle->slave_addr, 1);
    }
  }

  /* ── ADDR Event ──────────────────────────────────────────────── */
  if (evt_en && (sr1 & I2C_SR1_ADDR)) {
    I2C_ClearADDRFlag(pHandle);
  }

  /* ── BTF (Byte Transfer Finished) ────────────────────────────── */
  if (evt_en && (sr1 & I2C_SR1_BTF)) {
    if (pHandle->state == XFER_STATE_BUSY_TX) {
      if (pHandle->tx_len == 0) {
        /* All data sent — generate STOP or repeated START */
        if (pHandle->repeated_start == I2C_REPEATED_START_DISABLE) {
          I2C_GenerateStopCondition(pI2Cx);
        }
        I2C_CloseSendData(pHandle);

        if (pHandle->callback) {
          pHandle->callback(pHandle->cb_context, EVT_TX_COMPLETE);
        }
      }
    }
  }

  /* ── TXE (Transmit Buffer Empty) ─────────────────────────────── */
  if (evt_en && buf_en && (sr1 & I2C_SR1_TXE)) {
    if (pHandle->state == XFER_STATE_BUSY_TX) {
      if (pHandle->tx_len > 0) {
        pI2Cx->DR = *pHandle->pTxBuffer;
        pHandle->pTxBuffer++;
        pHandle->tx_len--;
      }
    }
  }

  /* ── RXNE (Receive Buffer Not Empty) ─────────────────────────── */
  if (evt_en && buf_en && (sr1 & I2C_SR1_RXNE)) {
    if (pHandle->state == XFER_STATE_BUSY_RX) {
      if (pHandle->rx_len == 1) {
        *pHandle->pRxBuffer = (uint8_t)pI2Cx->DR;
        pHandle->rx_len--;
      } else if (pHandle->rx_len > 1) {
        if (pHandle->rx_len == 2) {
          I2C_ManageACK(pI2Cx, DISABLE);
        }
        *pHandle->pRxBuffer = (uint8_t)pI2Cx->DR;
        pHandle->pRxBuffer++;
        pHandle->rx_len--;
      }

      if (pHandle->rx_len == 0) {
        if (pHandle->repeated_start == I2C_REPEATED_START_DISABLE) {
          I2C_GenerateStopCondition(pI2Cx);
        }
        I2C_CloseReceiveData(pHandle);

        if (pHandle->callback) {
          pHandle->callback(pHandle->cb_context, EVT_RX_COMPLETE);
        }
      }
    }
  }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Error IRQ Handler
 * ═══════════════════════════════════════════════════════════════════════════
 */

void I2C_ER_IRQHandler(I2C_Handle_t *pHandle) {
  I2C_RegDef_t *pI2Cx = pHandle->pI2Cx;
  uint32_t sr1 = pI2Cx->SR1;

  /* Bus Error */
  if (sr1 & I2C_SR1_BERR) {
    pI2Cx->SR1 &= ~I2C_SR1_BERR;
    if (pHandle->callback) {
      pHandle->callback(pHandle->cb_context, EVT_ERROR);
    }
  }

  /* Arbitration Lost */
  if (sr1 & I2C_SR1_ARLO) {
    pI2Cx->SR1 &= ~I2C_SR1_ARLO;
    if (pHandle->callback) {
      pHandle->callback(pHandle->cb_context, EVT_ERROR);
    }
  }

  /* ACK Failure */
  if (sr1 & I2C_SR1_AF) {
    pI2Cx->SR1 &= ~I2C_SR1_AF;
    /* In master mode, this means the slave did not acknowledge */
    I2C_GenerateStopCondition(pI2Cx);
    if (pHandle->callback) {
      pHandle->callback(pHandle->cb_context, EVT_ERROR);
    }
  }

  /* Overrun Error */
  if (sr1 & I2C_SR1_OVR) {
    pI2Cx->SR1 &= ~I2C_SR1_OVR;
    if (pHandle->callback) {
      pHandle->callback(pHandle->cb_context, EVT_ERROR);
    }
  }

  /* Timeout Error */
  if (sr1 & I2C_SR1_TIMEOUT) {
    pI2Cx->SR1 &= ~I2C_SR1_TIMEOUT;
    if (pHandle->callback) {
      pHandle->callback(pHandle->cb_context, EVT_ERROR);
    }
  }
}

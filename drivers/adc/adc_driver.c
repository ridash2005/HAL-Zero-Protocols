/**
 * @file    adc_driver.c
 * @brief   STM32F4xx ADC Driver — Implementation
 * @version 2.0.0
 */

#include "adc_driver.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  Clock Control
 * ═══════════════════════════════════════════════════════════════════════════
 */

void ADC_PeriClockControl(uint8_t en) {
  if (en == ENABLE) {
    ADC1_PCLK_EN();
  } else {
    ADC1_PCLK_DI();
  }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Initialization
 * ═══════════════════════════════════════════════════════════════════════════
 */

drv_status_t ADC_Init(ADC_Handle_t *pHandle) {
  if (pHandle == NULL) {
    return DRV_INVALID_PARAM;
  }

  ADC_RegDef_t *pADCx = pHandle->pADCx;
  ADC_Config_t *cfg = &pHandle->config;

  /* Enable ADC clock */
  ADC_PeriClockControl(ENABLE);

  /* 1. Set prescaler in common control register */
  ADC_COMMON->CCR &= ~ADC_CCR_ADCPRE_MASK;
  ADC_COMMON->CCR |= ((uint32_t)cfg->prescaler << 16);

  /* 2. Configure CR1 — resolution, scan mode */
  pADCx->CR1 &= ~ADC_CR1_RES_MASK;
  pADCx->CR1 |= ((uint32_t)cfg->resolution << 24);

  if (cfg->scan_mode) {
    pADCx->CR1 |= ADC_CR1_SCAN;
  } else {
    pADCx->CR1 &= ~ADC_CR1_SCAN;
  }

  /* 3. Configure CR2 — continuous mode, alignment */
  if (cfg->continuous) {
    pADCx->CR2 |= ADC_CR2_CONT;
  } else {
    pADCx->CR2 &= ~ADC_CR2_CONT;
  }

  if (cfg->alignment == ADC_ALIGN_LEFT) {
    pADCx->CR2 |= ADC_CR2_ALIGN;
  } else {
    pADCx->CR2 &= ~ADC_CR2_ALIGN;
  }

  /* Set EOC selection to per-channel (EOCS = 1) for single conversions */
  pADCx->CR2 |= ADC_CR2_EOCS;

  /* 4. Set number of conversions in regular sequence */
  pADCx->SQR1 &= ~(0xF << 20);
  if (cfg->num_channels > 0) {
    pADCx->SQR1 |= ((uint32_t)(cfg->num_channels - 1) << 20);
  }

  return DRV_OK;
}

drv_status_t ADC_DeInit(ADC_RegDef_t *pADCx) {
  UNUSED(pADCx);
  /* Reset via RCC */
  ADC_PeriClockControl(DISABLE);
  return DRV_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Channel Configuration
 * ═══════════════════════════════════════════════════════════════════════════
 */

/**
 * @brief  Configure a channel in the regular sequence.
 * @param  channel: ADC channel number (0-18)
 * @param  rank: Position in the conversion sequence (1-16)
 * @param  sample_time: Sampling time for this channel
 */
void ADC_ConfigChannel(ADC_RegDef_t *pADCx, adc_channel_t channel, uint8_t rank,
                       adc_sample_time_t sample_time) {
  /* Set sample time */
  if (channel <= ADC_CHANNEL_9) {
    /* SMPR2 covers channels 0-9 */
    uint8_t pos = channel * 3;
    pADCx->SMPR2 &= ~(0x7 << pos);
    pADCx->SMPR2 |= ((uint32_t)sample_time << pos);
  } else {
    /* SMPR1 covers channels 10-18 */
    uint8_t pos = (channel - 10) * 3;
    pADCx->SMPR1 &= ~(0x7 << pos);
    pADCx->SMPR1 |= ((uint32_t)sample_time << pos);
  }

  /* Set channel in conversion sequence */
  if (rank <= 6) {
    /* SQR3 covers ranks 1-6 */
    uint8_t pos = (rank - 1) * 5;
    pADCx->SQR3 &= ~(0x1F << pos);
    pADCx->SQR3 |= ((uint32_t)channel << pos);
  } else if (rank <= 12) {
    /* SQR2 covers ranks 7-12 */
    uint8_t pos = (rank - 7) * 5;
    pADCx->SQR2 &= ~(0x1F << pos);
    pADCx->SQR2 |= ((uint32_t)channel << pos);
  } else {
    /* SQR1 covers ranks 13-16 */
    uint8_t pos = (rank - 13) * 5;
    pADCx->SQR1 &= ~(0x1F << pos);
    pADCx->SQR1 |= ((uint32_t)channel << pos);
  }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Enable / Disable
 * ═══════════════════════════════════════════════════════════════════════════
 */

void ADC_Enable(ADC_RegDef_t *pADCx) { pADCx->CR2 |= ADC_CR2_ADON; }

void ADC_Disable(ADC_RegDef_t *pADCx) { pADCx->CR2 &= ~ADC_CR2_ADON; }

/* ═══════════════════════════════════════════════════════════════════════════
 *  Polling (Blocking) Conversion
 * ═══════════════════════════════════════════════════════════════════════════
 */

drv_status_t ADC_StartConversion(ADC_RegDef_t *pADCx) {
  /* Clear status register */
  pADCx->SR = 0;

  /* Start conversion of regular channels */
  pADCx->CR2 |= ADC_CR2_SWSTART;

  return DRV_OK;
}

uint16_t ADC_ReadValue(ADC_RegDef_t *pADCx) {
  /* Wait for EOC */
  while (!(pADCx->SR & ADC_SR_EOC))
    ;

  return (uint16_t)(pADCx->DR);
}

/**
 * @brief  Read a single channel (convenience function).
 *         Configures channel, starts conversion, and returns the result.
 */
uint16_t ADC_ReadChannel(ADC_Handle_t *pHandle, adc_channel_t channel,
                         adc_sample_time_t sample_time) {
  ADC_RegDef_t *pADCx = pHandle->pADCx;

  /* Configure channel as rank 1, single conversion */
  ADC_ConfigChannel(pADCx, channel, 1, sample_time);

  /* Ensure single conversion mode */
  pADCx->CR2 &= ~ADC_CR2_CONT;
  pADCx->SQR1 &= ~(0xF << 20); /* 1 conversion */

  /* Enable ADC and start */
  ADC_Enable(pADCx);
  ADC_StartConversion(pADCx);

  /* Wait and read */
  return ADC_ReadValue(pADCx);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Interrupt (Non-Blocking) Conversion
 * ═══════════════════════════════════════════════════════════════════════════
 */

drv_status_t ADC_StartConversion_IT(ADC_Handle_t *pHandle) {
  if (pHandle == NULL) {
    return DRV_INVALID_PARAM;
  }

  /* Enable EOC interrupt */
  pHandle->pADCx->CR1 |= ADC_CR1_EOCIE;

  /* Enable overrun interrupt */
  pHandle->pADCx->CR1 |= ADC_CR1_OVRIE;

  /* Enable ADC */
  ADC_Enable(pHandle->pADCx);

  /* Start conversion */
  pHandle->pADCx->CR2 |= ADC_CR2_SWSTART;

  return DRV_OK;
}

void ADC_IRQHandler(ADC_Handle_t *pHandle) {
  ADC_RegDef_t *pADCx = pHandle->pADCx;

  if (pADCx->SR & ADC_SR_EOC) {
    /* Read the converted data */
    uint16_t value = (uint16_t)(pADCx->DR);

    if (pHandle->callback) {
      pHandle->callback(pHandle->cb_context, value);
    }

    /* In continuous mode, the next conversion starts automatically */
  }

  if (pADCx->SR & ADC_SR_OVR) {
    /* Overrun error — clear flag */
    pADCx->SR &= ~ADC_SR_OVR;
  }
}

void ADC_RegisterCallback(ADC_Handle_t *pHandle, adc_callback_t cb, void *ctx) {
  pHandle->callback = cb;
  pHandle->cb_context = ctx;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  DMA Continuous Sampling
 * ═══════════════════════════════════════════════════════════════════════════
 */

drv_status_t ADC_StartContinuousDMA(ADC_Handle_t *pHandle, uint16_t *pBuffer,
                                    uint32_t len) {
  if (pHandle == NULL || pBuffer == NULL || len == 0) {
    return DRV_INVALID_PARAM;
  }

  ADC_RegDef_t *pADCx = pHandle->pADCx;

  /*
   * DMA Configuration for ADC:
   * ADC1 → DMA2 Stream 0, Channel 0  (or DMA2 Stream 4, Channel 0)
   *
   * The user should configure DMA with:
   *  - Peripheral address = &ADC1->DR
   *  - Memory address = pBuffer
   *  - Data count = len
   *  - Direction = Peripheral-to-Memory
   *  - Data size = Half-word (16-bit)
   *  - Circular mode = enabled
   *  - Memory increment = enabled
   */

  /* Enable continuous conversion */
  pADCx->CR2 |= ADC_CR2_CONT;

  /* Enable DMA access and select DDS for continuous DMA requests */
  pADCx->CR2 |= (ADC_CR2_DMA | ADC_CR2_DDS);

  /* Enable ADC */
  ADC_Enable(pADCx);

  /* Start conversion */
  pADCx->CR2 |= ADC_CR2_SWSTART;

  UNUSED(pBuffer);
  UNUSED(len);

  return DRV_OK;
}

void ADC_StopContinuousDMA(ADC_Handle_t *pHandle) {
  ADC_RegDef_t *pADCx = pHandle->pADCx;

  /* Stop conversion */
  pADCx->CR2 &= ~ADC_CR2_CONT;
  pADCx->CR2 &= ~ADC_CR2_DMA;
  pADCx->CR2 &= ~ADC_CR2_DDS;

  ADC_Disable(pADCx);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Temperature Sensor
 * ═══════════════════════════════════════════════════════════════════════════
 */

void ADC_EnableTempSensor(void) {
  /* Enable internal temperature sensor and VREFINT (channel 16, 17) */
  ADC_COMMON->CCR |= ADC_CCR_TSVREFE;

  /* Wait for stabilization (~10 µs) */
  for (volatile int i = 0; i < 1000; i++)
    ;
}

uint16_t ADC_ReadTemperature(ADC_Handle_t *pHandle) {
  ADC_EnableTempSensor();

  /* Temperature sensor is on channel 18 (STM32F4) */
  return ADC_ReadChannel(pHandle, ADC_CHANNEL_18, ADC_SAMPLETIME_480);
}

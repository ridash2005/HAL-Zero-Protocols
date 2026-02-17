/**
 * @file    pwm_driver.c
 * @brief   STM32F4xx PWM Driver — Implementation
 * @version 2.0.0
 */

#include "pwm_driver.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  Internal Helpers
 * ═══════════════════════════════════════════════════════════════════════════
 */

/**
 * @brief Write to the correct CCR register for the given channel.
 */
static void PWM_WriteCCR(TIM_RegDef_t *pTIMx, tim_channel_t ch,
                         uint32_t value) {
  switch (ch) {
  case TIM_CHANNEL_1:
    pTIMx->CCR1 = value;
    break;
  case TIM_CHANNEL_2:
    pTIMx->CCR2 = value;
    break;
  case TIM_CHANNEL_3:
    pTIMx->CCR3 = value;
    break;
  case TIM_CHANNEL_4:
    pTIMx->CCR4 = value;
    break;
  }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Initialization
 * ═══════════════════════════════════════════════════════════════════════════
 */

drv_status_t PWM_Init(PWM_Handle_t *pHandle) {
  if (pHandle == NULL || pHandle->config.frequency_hz == 0) {
    return DRV_INVALID_PARAM;
  }

  TIM_RegDef_t *pTIMx = pHandle->config.pTIMx;
  tim_channel_t ch = pHandle->config.channel;

  /* Enable timer clock */
  TIM_PeriClockControl(pTIMx, ENABLE);

  /* Calculate PSC and ARR for desired frequency
   *   Timer frequency = SYSTEM_CLOCK_HZ / (PSC + 1)
   *   PWM frequency   = Timer_freq / (ARR + 1)
   *
   * We target ARR = 999 for good duty resolution (0.1% steps),
   * then compute PSC accordingly.
   * If that doesn't divide evenly, we try ARR = 65535.
   */
  uint32_t target_arr = 999;
  uint32_t psc =
      (SYSTEM_CLOCK_HZ / ((target_arr + 1) * pHandle->config.frequency_hz)) - 1;

  if (psc > 0xFFFF) {
    /* Frequency too low for ARR=999, use maximum ARR */
    target_arr = 0xFFFF;
    psc =
        (SYSTEM_CLOCK_HZ / ((target_arr + 1) * pHandle->config.frequency_hz)) -
        1;
  }

  pHandle->arr_value = target_arr;

  /* Configure timer base */
  pTIMx->PSC = psc;
  pTIMx->ARR = target_arr;
  pTIMx->CR1 |= TIM_CR1_ARPE; /* Auto-reload preload */

  /* Configure output compare mode */
  uint32_t oc_mode = (uint32_t)pHandle->config.mode;

  switch (ch) {
  case TIM_CHANNEL_1:
    pTIMx->CCMR1 &= ~TIM_CCMR1_OC1M_MASK;
    pTIMx->CCMR1 |= (oc_mode << 4);
    pTIMx->CCMR1 |= TIM_CCMR1_OC1PE; /* Output compare preload */
    break;
  case TIM_CHANNEL_2:
    pTIMx->CCMR1 &= ~TIM_CCMR1_OC2M_MASK;
    pTIMx->CCMR1 |= (oc_mode << 12);
    pTIMx->CCMR1 |= TIM_CCMR1_OC2PE;
    break;
  case TIM_CHANNEL_3:
    pTIMx->CCMR2 &= ~(0x7 << 4);
    pTIMx->CCMR2 |= (oc_mode << 4);
    pTIMx->CCMR2 |= (1 << 3); /* OC3PE */
    break;
  case TIM_CHANNEL_4:
    pTIMx->CCMR2 &= ~(0x7 << 12);
    pTIMx->CCMR2 |= (oc_mode << 12);
    pTIMx->CCMR2 |= (1 << 11); /* OC4PE */
    break;
  }

  /* Configure output polarity */
  uint32_t ccer_bit;
  switch (ch) {
  case TIM_CHANNEL_1:
    ccer_bit = TIM_CCER_CC1P;
    break;
  case TIM_CHANNEL_2:
    ccer_bit = TIM_CCER_CC2P;
    break;
  case TIM_CHANNEL_3:
    ccer_bit = TIM_CCER_CC3P;
    break;
  case TIM_CHANNEL_4:
    ccer_bit = TIM_CCER_CC4P;
    break;
  default:
    ccer_bit = 0;
    break;
  }

  if (pHandle->config.polarity == PWM_POLARITY_LOW) {
    pTIMx->CCER |= ccer_bit;
  } else {
    pTIMx->CCER &= ~ccer_bit;
  }

  /* Set duty cycle */
  pHandle->ccr_value =
      (pHandle->arr_value * pHandle->config.duty_percent) / 100;
  PWM_WriteCCR(pTIMx, ch, pHandle->ccr_value);

  /* Generate update event to load registers */
  pTIMx->EGR = TIM_EGR_UG;
  pTIMx->SR &= ~TIM_SR_UIF;

  return DRV_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Start / Stop
 * ═══════════════════════════════════════════════════════════════════════════
 */

void PWM_Start(PWM_Handle_t *pHandle) {
  TIM_RegDef_t *pTIMx = pHandle->config.pTIMx;
  tim_channel_t ch = pHandle->config.channel;

  /* Enable output compare channel */
  switch (ch) {
  case TIM_CHANNEL_1:
    pTIMx->CCER |= TIM_CCER_CC1E;
    break;
  case TIM_CHANNEL_2:
    pTIMx->CCER |= TIM_CCER_CC2E;
    break;
  case TIM_CHANNEL_3:
    pTIMx->CCER |= TIM_CCER_CC3E;
    break;
  case TIM_CHANNEL_4:
    pTIMx->CCER |= TIM_CCER_CC4E;
    break;
  }

  /* For advanced timers (TIM1), the MOE bit in BDTR must be set */
  if (pTIMx == TIM1) {
    pTIMx->BDTR |= (1 << 15); /* MOE — Main Output Enable */
  }

  /* Start the counter */
  pTIMx->CR1 |= TIM_CR1_CEN;
}

void PWM_Stop(PWM_Handle_t *pHandle) {
  TIM_RegDef_t *pTIMx = pHandle->config.pTIMx;
  tim_channel_t ch = pHandle->config.channel;

  /* Disable output compare channel */
  switch (ch) {
  case TIM_CHANNEL_1:
    pTIMx->CCER &= ~TIM_CCER_CC1E;
    break;
  case TIM_CHANNEL_2:
    pTIMx->CCER &= ~TIM_CCER_CC2E;
    break;
  case TIM_CHANNEL_3:
    pTIMx->CCER &= ~TIM_CCER_CC3E;
    break;
  case TIM_CHANNEL_4:
    pTIMx->CCER &= ~TIM_CCER_CC4E;
    break;
  }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Runtime Adjustment
 * ═══════════════════════════════════════════════════════════════════════════
 */

void PWM_SetDuty(PWM_Handle_t *pHandle, uint8_t duty_percent) {
  if (duty_percent > 100) {
    duty_percent = 100;
  }

  pHandle->config.duty_percent = duty_percent;
  pHandle->ccr_value = (pHandle->arr_value * duty_percent) / 100;
  PWM_WriteCCR(pHandle->config.pTIMx, pHandle->config.channel,
               pHandle->ccr_value);
}

void PWM_SetFrequency(PWM_Handle_t *pHandle, uint32_t frequency_hz) {
  if (frequency_hz == 0)
    return;

  pHandle->config.frequency_hz = frequency_hz;

  uint32_t target_arr = 999;
  uint32_t psc = (SYSTEM_CLOCK_HZ / ((target_arr + 1) * frequency_hz)) - 1;

  if (psc > 0xFFFF) {
    target_arr = 0xFFFF;
    psc = (SYSTEM_CLOCK_HZ / ((target_arr + 1) * frequency_hz)) - 1;
  }

  pHandle->arr_value = target_arr;
  pHandle->config.pTIMx->PSC = psc;
  pHandle->config.pTIMx->ARR = target_arr;

  /* Recalculate CCR for current duty */
  PWM_SetDuty(pHandle, pHandle->config.duty_percent);
}

uint8_t PWM_GetDuty(PWM_Handle_t *pHandle) {
  return pHandle->config.duty_percent;
}

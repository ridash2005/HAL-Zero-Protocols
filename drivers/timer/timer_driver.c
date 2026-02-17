/**
 * @file    timer_driver.c
 * @brief   STM32F4xx Timer Driver — Implementation
 * @version 2.0.0
 */

#include "timer_driver.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  Clock Control
 * ═══════════════════════════════════════════════════════════════════════════
 */

void TIM_PeriClockControl(TIM_RegDef_t *pTIMx, uint8_t en) {
  if (en == ENABLE) {
    if (pTIMx == TIM2) {
      TIM2_PCLK_EN();
    } else if (pTIMx == TIM3) {
      TIM3_PCLK_EN();
    } else if (pTIMx == TIM4) {
      TIM4_PCLK_EN();
    } else if (pTIMx == TIM5) {
      TIM5_PCLK_EN();
    } else if (pTIMx == TIM1) {
      TIM1_PCLK_EN();
    }
  } else {
    if (pTIMx == TIM2) {
      TIM2_PCLK_DI();
    } else if (pTIMx == TIM3) {
      TIM3_PCLK_DI();
    } else if (pTIMx == TIM4) {
      TIM4_PCLK_DI();
    } else if (pTIMx == TIM5) {
      TIM5_PCLK_DI();
    } else if (pTIMx == TIM1) {
      TIM1_PCLK_DI();
    }
  }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Initialization
 * ═══════════════════════════════════════════════════════════════════════════
 */

drv_status_t TIM_Init(TIM_Handle_t *pHandle) {
  if (pHandle == NULL) {
    return DRV_INVALID_PARAM;
  }

  TIM_RegDef_t *pTIMx = pHandle->pTIMx;
  TIM_Config_t *cfg = &pHandle->config;

  /* Enable peripheral clock */
  TIM_PeriClockControl(pTIMx, ENABLE);

  /* Configure CR1 */
  uint32_t cr1 = 0;

  /* Counting direction */
  if (cfg->direction == TIM_COUNT_DOWN) {
    cr1 |= TIM_CR1_DIR;
  }

  /* One-pulse mode */
  if (cfg->one_pulse) {
    cr1 |= TIM_CR1_OPM;
  }

  /* Auto-reload preload */
  if (cfg->auto_reload_preload) {
    cr1 |= TIM_CR1_ARPE;
  }

  pTIMx->CR1 = cr1;

  /* Set prescaler and auto-reload */
  pTIMx->PSC = cfg->prescaler;
  pTIMx->ARR = cfg->period;

  /* Generate an update event to load PSC and ARR immediately */
  pTIMx->EGR = TIM_EGR_UG;

  /* Clear the update interrupt flag that EGR sets */
  pTIMx->SR &= ~TIM_SR_UIF;

  return DRV_OK;
}

drv_status_t TIM_DeInit(TIM_RegDef_t *pTIMx) {
  /* Use RCC reset to return to defaults */
  TIM_PeriClockControl(pTIMx, DISABLE);
  return DRV_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Counter Control
 * ═══════════════════════════════════════════════════════════════════════════
 */

void TIM_Start(TIM_RegDef_t *pTIMx) { pTIMx->CR1 |= TIM_CR1_CEN; }

void TIM_Stop(TIM_RegDef_t *pTIMx) { pTIMx->CR1 &= ~TIM_CR1_CEN; }

uint32_t TIM_GetCounter(TIM_RegDef_t *pTIMx) { return pTIMx->CNT; }

void TIM_SetCounter(TIM_RegDef_t *pTIMx, uint32_t value) { pTIMx->CNT = value; }

/* ═══════════════════════════════════════════════════════════════════════════
 *  Interrupt
 * ═══════════════════════════════════════════════════════════════════════════
 */

void TIM_EnableUpdateInterrupt(TIM_Handle_t *pHandle) {
  pHandle->pTIMx->DIER |= TIM_DIER_UIE;
}

void TIM_DisableUpdateInterrupt(TIM_RegDef_t *pTIMx) {
  pTIMx->DIER &= ~TIM_DIER_UIE;
}

void TIM_IRQHandler(TIM_Handle_t *pHandle) {
  if (pHandle->pTIMx->SR & TIM_SR_UIF) {
    /* Clear the update interrupt flag */
    pHandle->pTIMx->SR &= ~TIM_SR_UIF;

    if (pHandle->callback) {
      pHandle->callback(pHandle->cb_context);
    }
  }
}

void TIM_RegisterCallback(TIM_Handle_t *pHandle, timer_callback_t cb,
                          void *ctx) {
  pHandle->callback = cb;
  pHandle->cb_context = ctx;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Delay Functions (Blocking — uses TIM5, 32-bit counter)
 * ═══════════════════════════════════════════════════════════════════════════
 */

/**
 * @brief  Microsecond delay using TIM5.
 * @note   Configures TIM5 internally. Do not use TIM5 for other purposes
 *         simultaneously when using this function.
 */
void TIM_DelayUS(uint32_t us) {
  TIM5_PCLK_EN();

  /* Prescaler: clock_freq / 1MHz - 1 → 1 µs per tick */
  TIM5->PSC = (SYSTEM_CLOCK_HZ / 1000000UL) - 1;
  TIM5->ARR = us;
  TIM5->CNT = 0;
  TIM5->EGR = TIM_EGR_UG; /* Load prescaler immediately */
  TIM5->SR &= ~TIM_SR_UIF;
  TIM5->CR1 |= (TIM_CR1_OPM | TIM_CR1_CEN); /* One-pulse, start */

  /* Wait for update flag */
  while (!(TIM5->SR & TIM_SR_UIF))
    ;

  TIM5->SR &= ~TIM_SR_UIF;
  TIM5->CR1 &= ~TIM_CR1_CEN;
}

void TIM_DelayMS(uint32_t ms) {
  while (ms > 0) {
    TIM_DelayUS(1000);
    ms--;
  }
}

/**
 * @file    timer_driver.h
 * @brief   STM32F4xx Timer Driver — Public API
 * @version 2.0.0
 *
 * @details General purpose timer driver for TIM2-TIM5 supporting:
 *          - Up-counting and Down-counting modes
 *          - One-pulse mode
 *          - Update interrupt with callback
 *          - Blocking delay functions (µs, ms) using TIM5
 *          - PWM output (via PWM driver)
 */

#ifndef TIMER_DRIVER_H
#define TIMER_DRIVER_H

#include "../common/error_codes.h"
#include "../common/stm32f4xx_base.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 *  Timer Register Bit Definitions
 * ═══════════════════════════════════════════════════════════════════════════
 */

/* CR1 */
#define TIM_CR1_CEN (1 << 0)        /*!< Counter enable            */
#define TIM_CR1_UDIS (1 << 1)       /*!< Update disable            */
#define TIM_CR1_URS (1 << 2)        /*!< Update request source     */
#define TIM_CR1_OPM (1 << 3)        /*!< One-pulse mode            */
#define TIM_CR1_DIR (1 << 4)        /*!< Direction (0=up, 1=down)  */
#define TIM_CR1_CMS_MASK (0x3 << 5) /*!< Center-aligned mode       */
#define TIM_CR1_ARPE (1 << 7)       /*!< Auto-reload preload       */

/* DIER */
#define TIM_DIER_UIE (1 << 0)   /*!< Update interrupt enable   */
#define TIM_DIER_CC1IE (1 << 1) /*!< CC1 interrupt enable      */
#define TIM_DIER_CC2IE (1 << 2) /*!< CC2 interrupt enable      */
#define TIM_DIER_CC3IE (1 << 3) /*!< CC3 interrupt enable      */
#define TIM_DIER_CC4IE (1 << 4) /*!< CC4 interrupt enable      */
#define TIM_DIER_UDE (1 << 8)   /*!< Update DMA request enable */

/* SR */
#define TIM_SR_UIF (1 << 0) /*!< Update interrupt flag     */
#define TIM_SR_CC1IF (1 << 1)
#define TIM_SR_CC2IF (1 << 2)
#define TIM_SR_CC3IF (1 << 3)
#define TIM_SR_CC4IF (1 << 4)

/* EGR */
#define TIM_EGR_UG (1 << 0) /*!< Update generation         */

/* CCMR1 — Output Compare Mode */
#define TIM_CCMR1_OC1PE (1 << 3)       /*!< OC1 preload enable        */
#define TIM_CCMR1_OC1M_MASK (0x7 << 4) /*!< OC1 mode mask             */
#define TIM_CCMR1_OC2PE (1 << 11)
#define TIM_CCMR1_OC2M_MASK (0x7 << 12)

/* CCER */
#define TIM_CCER_CC1E (1 << 0) /*!< CC1 output enable         */
#define TIM_CCER_CC1P (1 << 1) /*!< CC1 output polarity       */
#define TIM_CCER_CC2E (1 << 4)
#define TIM_CCER_CC2P (1 << 5)
#define TIM_CCER_CC3E (1 << 8)
#define TIM_CCER_CC3P (1 << 9)
#define TIM_CCER_CC4E (1 << 12)
#define TIM_CCER_CC4P (1 << 13)

/* ═══════════════════════════════════════════════════════════════════════════
 *  Configuration Enumerations
 * ═══════════════════════════════════════════════════════════════════════════
 */

typedef enum { TIM_COUNT_UP = 0, TIM_COUNT_DOWN = 1 } tim_direction_t;

typedef enum {
  TIM_OC_MODE_FROZEN = 0x00,
  TIM_OC_MODE_ACTIVE = 0x01,
  TIM_OC_MODE_INACTIVE = 0x02,
  TIM_OC_MODE_TOGGLE = 0x03,
  TIM_OC_MODE_FORCE_LOW = 0x04,
  TIM_OC_MODE_FORCE_HIGH = 0x05,
  TIM_OC_MODE_PWM1 = 0x06,
  TIM_OC_MODE_PWM2 = 0x07
} tim_oc_mode_t;

typedef enum {
  TIM_CHANNEL_1 = 1,
  TIM_CHANNEL_2 = 2,
  TIM_CHANNEL_3 = 3,
  TIM_CHANNEL_4 = 4
} tim_channel_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Handle Structures
 * ═══════════════════════════════════════════════════════════════════════════
 */

typedef void (*timer_callback_t)(void *context);

typedef struct {
  uint32_t prescaler;          /*!< Prescaler value (PSC)                 */
  uint32_t period;             /*!< Auto-reload value (ARR)               */
  tim_direction_t direction;   /*!< Counting direction                    */
  uint8_t one_pulse;           /*!< One-pulse mode (0 or 1)               */
  uint8_t auto_reload_preload; /*!< ARR preload enable (0 or 1)      */
} TIM_Config_t;

typedef struct {
  TIM_RegDef_t *pTIMx;       /*!< Timer peripheral base address         */
  TIM_Config_t config;       /*!< Timer configuration                   */
  timer_callback_t callback; /*!< Update event callback                 */
  void *cb_context;          /*!< User context for callback             */
} TIM_Handle_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════════════
 */

/* Initialization */
drv_status_t TIM_Init(TIM_Handle_t *pHandle);
drv_status_t TIM_DeInit(TIM_RegDef_t *pTIMx);
void TIM_PeriClockControl(TIM_RegDef_t *pTIMx, uint8_t en);

/* Counter Control */
void TIM_Start(TIM_RegDef_t *pTIMx);
void TIM_Stop(TIM_RegDef_t *pTIMx);
uint32_t TIM_GetCounter(TIM_RegDef_t *pTIMx);
void TIM_SetCounter(TIM_RegDef_t *pTIMx, uint32_t value);

/* Interrupt */
void TIM_EnableUpdateInterrupt(TIM_Handle_t *pHandle);
void TIM_DisableUpdateInterrupt(TIM_RegDef_t *pTIMx);
void TIM_IRQHandler(TIM_Handle_t *pHandle);

/* Delay (uses TIM5 — 32-bit counter) */
void TIM_DelayUS(uint32_t us);
void TIM_DelayMS(uint32_t ms);

/* Callback */
void TIM_RegisterCallback(TIM_Handle_t *pHandle, timer_callback_t cb,
                          void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* TIMER_DRIVER_H */

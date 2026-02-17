/**
 * @file    pwm_driver.h
 * @brief   STM32F4xx PWM Driver — Public API
 * @version 2.0.0
 *
 * @details PWM output driver built on top of the Timer peripheral.
 *          - Configurable frequency and duty cycle
 *          - Supports 4 channels per timer (CH1-CH4)
 *          - PWM Mode 1 and Mode 2
 *          - Runtime duty cycle adjustment
 */

#ifndef PWM_DRIVER_H
#define PWM_DRIVER_H

#include "../common/error_codes.h"
#include "../common/stm32f4xx_base.h"
#include "../timer/timer_driver.h"


#ifdef __cplusplus
extern "C" {
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 *  Configuration
 * ═══════════════════════════════════════════════════════════════════════════
 */

typedef enum {
  PWM_MODE_1 = TIM_OC_MODE_PWM1, /*!< Active when CNT < CCR     */
  PWM_MODE_2 = TIM_OC_MODE_PWM2  /*!< Active when CNT > CCR     */
} pwm_mode_t;

typedef enum {
  PWM_POLARITY_HIGH = 0, /*!< Active high output            */
  PWM_POLARITY_LOW = 1   /*!< Active low output             */
} pwm_polarity_t;

typedef struct {
  TIM_RegDef_t *pTIMx;     /*!< Timer to use for PWM              */
  tim_channel_t channel;   /*!< Output channel (1-4)              */
  uint32_t frequency_hz;   /*!< Desired PWM frequency in Hz       */
  uint8_t duty_percent;    /*!< Duty cycle (0-100%)               */
  pwm_mode_t mode;         /*!< PWM mode (1 or 2)                 */
  pwm_polarity_t polarity; /*!< Output polarity                   */
} PWM_Config_t;

typedef struct {
  PWM_Config_t config;
  uint32_t arr_value; /*!< Computed ARR value                */
  uint32_t ccr_value; /*!< Computed CCR value                */
} PWM_Handle_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════════════
 */

/**
 * @brief  Initialize PWM output on the specified timer channel.
 * @param  pHandle: PWM handle with desired configuration
 * @retval drv_status_t
 */
drv_status_t PWM_Init(PWM_Handle_t *pHandle);

/**
 * @brief  Start the PWM output.
 * @param  pHandle: Initialized PWM handle
 */
void PWM_Start(PWM_Handle_t *pHandle);

/**
 * @brief  Stop the PWM output.
 * @param  pHandle: PWM handle
 */
void PWM_Stop(PWM_Handle_t *pHandle);

/**
 * @brief  Update the duty cycle at runtime (0-100%).
 * @param  pHandle: PWM handle
 * @param  duty_percent: New duty cycle percentage
 */
void PWM_SetDuty(PWM_Handle_t *pHandle, uint8_t duty_percent);

/**
 * @brief  Update the PWM frequency at runtime.
 * @param  pHandle: PWM handle
 * @param  frequency_hz: New frequency in Hz
 */
void PWM_SetFrequency(PWM_Handle_t *pHandle, uint32_t frequency_hz);

/**
 * @brief  Get the current duty cycle setting.
 * @param  pHandle: PWM handle
 * @retval Current duty percentage (0-100)
 */
uint8_t PWM_GetDuty(PWM_Handle_t *pHandle);

#ifdef __cplusplus
}
#endif

#endif /* PWM_DRIVER_H */

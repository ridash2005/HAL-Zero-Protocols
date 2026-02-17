/**
 * @file    gpio_driver.h
 * @brief   STM32F4xx GPIO Driver — Public API
 * @version 2.0.0
 *
 * @details Full-featured GPIO driver supporting:
 *          - Input, Output, Alternate Function, and Analog modes
 *          - Push-pull / Open-drain output types
 *          - Speed selection (Low / Medium / Fast / High)
 *          - Pull-up / Pull-down configuration
 *          - EXTI interrupt support (Rising / Falling / Both edges)
 *          - Atomic bit set/reset via BSRR
 */

#ifndef GPIO_DRIVER_H
#define GPIO_DRIVER_H

#include "../common/error_codes.h"
#include "../common/stm32f4xx_base.h"


#ifdef __cplusplus
extern "C" {
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 *  Configuration Enumerations
 * ═══════════════════════════════════════════════════════════════════════════
 */

/** @brief GPIO pin numbers */
typedef enum {
  GPIO_PIN_0 = 0,
  GPIO_PIN_1,
  GPIO_PIN_2,
  GPIO_PIN_3,
  GPIO_PIN_4,
  GPIO_PIN_5,
  GPIO_PIN_6,
  GPIO_PIN_7,
  GPIO_PIN_8,
  GPIO_PIN_9,
  GPIO_PIN_10,
  GPIO_PIN_11,
  GPIO_PIN_12,
  GPIO_PIN_13,
  GPIO_PIN_14,
  GPIO_PIN_15
} gpio_pin_t;

/** @brief GPIO mode */
typedef enum {
  GPIO_MODE_INPUT = 0x00,  /*!< Input mode (reset state)     */
  GPIO_MODE_OUTPUT = 0x01, /*!< General purpose output       */
  GPIO_MODE_ALTFN = 0x02,  /*!< Alternate function           */
  GPIO_MODE_ANALOG = 0x03, /*!< Analog mode                  */
  GPIO_MODE_IT_FT = 0x04,  /*!< Interrupt, falling edge      */
  GPIO_MODE_IT_RT = 0x05,  /*!< Interrupt, rising edge       */
  GPIO_MODE_IT_RFT = 0x06, /*!< Interrupt, rising + falling  */
} gpio_mode_t;

/** @brief Output type */
typedef enum { GPIO_OTYPE_PUSHPULL = 0, GPIO_OTYPE_OPENDRAIN = 1 } gpio_otype_t;

/** @brief Output speed */
typedef enum {
  GPIO_SPEED_LOW = 0x00,
  GPIO_SPEED_MEDIUM = 0x01,
  GPIO_SPEED_FAST = 0x02,
  GPIO_SPEED_HIGH = 0x03
} gpio_speed_t;

/** @brief Pull-up / Pull-down */
typedef enum {
  GPIO_PUPD_NONE = 0x00,
  GPIO_PUPD_PULLUP = 0x01,
  GPIO_PUPD_PULLDOWN = 0x02
} gpio_pupd_t;

/** @brief Alternate function number (AF0-AF15) */
typedef enum {
  GPIO_AF0 = 0,
  GPIO_AF1,
  GPIO_AF2,
  GPIO_AF3,
  GPIO_AF4,
  GPIO_AF5,
  GPIO_AF6,
  GPIO_AF7,
  GPIO_AF8,
  GPIO_AF9,
  GPIO_AF10,
  GPIO_AF11,
  GPIO_AF12,
  GPIO_AF13,
  GPIO_AF14,
  GPIO_AF15
} gpio_altfn_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Handle Structures
 * ═══════════════════════════════════════════════════════════════════════════
 */

/** @brief GPIO pin configuration */
typedef struct {
  gpio_pin_t pin_number;     /*!< Pin number (0-15)                 */
  gpio_mode_t mode;          /*!< Pin mode                          */
  gpio_speed_t speed;        /*!< Output speed                      */
  gpio_otype_t output_type;  /*!< Output type (PP/OD)               */
  gpio_pupd_t pull;          /*!< Pull-up / Pull-down               */
  gpio_altfn_t alt_function; /*!< Alternate function (AF0-AF15)     */
} GPIO_PinConfig_t;

/** @brief GPIO handle (reference to port + pin configuration) */
typedef struct {
  GPIO_RegDef_t *pGPIOx;   /*!< Pointer to GPIO port base address */
  GPIO_PinConfig_t config; /*!< Pin configuration settings        */
} GPIO_Handle_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════════════
 */

/**
 * @brief  Initialize a GPIO pin based on the handle configuration.
 * @param  pHandle: Pointer to GPIO handle structure
 * @retval drv_status_t
 */
drv_status_t GPIO_Init(GPIO_Handle_t *pHandle);

/**
 * @brief  De-initialize a GPIO port (reset to defaults).
 * @param  pGPIOx: Pointer to GPIO port register structure
 * @retval drv_status_t
 */
drv_status_t GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/**
 * @brief  Enable or disable the peripheral clock for a GPIO port.
 * @param  pGPIOx: GPIO port base address
 * @param  en: ENABLE or DISABLE
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t en);

/**
 * @brief  Read a single input pin.
 * @param  pGPIOx: GPIO port base address
 * @param  pin: Pin number
 * @retval uint8_t: 0 or 1
 */
uint8_t GPIO_ReadPin(GPIO_RegDef_t *pGPIOx, gpio_pin_t pin);

/**
 * @brief  Read the entire 16-bit input port.
 * @param  pGPIOx: GPIO port base address
 * @retval uint16_t: Port value
 */
uint16_t GPIO_ReadPort(GPIO_RegDef_t *pGPIOx);

/**
 * @brief  Write a value to a single output pin.
 * @param  pGPIOx: GPIO port base address
 * @param  pin: Pin number
 * @param  value: 0 (reset) or 1 (set)
 */
void GPIO_WritePin(GPIO_RegDef_t *pGPIOx, gpio_pin_t pin, uint8_t value);

/**
 * @brief  Write a 16-bit value to the entire output port.
 * @param  pGPIOx: GPIO port base address
 * @param  value: 16-bit port output value
 */
void GPIO_WritePort(GPIO_RegDef_t *pGPIOx, uint16_t value);

/**
 * @brief  Toggle an output pin using atomic BSRR access.
 * @param  pGPIOx: GPIO port base address
 * @param  pin: Pin number
 */
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, gpio_pin_t pin);

/**
 * @brief  Configure NVIC for a GPIO EXTI interrupt.
 * @param  irq_number: IRQ number
 * @param  en: ENABLE or DISABLE
 */
void GPIO_IRQConfig(IRQn_t irq_number, uint8_t en);

/**
 * @brief  Set the priority for a GPIO EXTI interrupt.
 * @param  irq_number: IRQ number
 * @param  priority: Priority level (0-15 for STM32F4)
 */
void GPIO_IRQPriorityConfig(IRQn_t irq_number, uint8_t priority);

/**
 * @brief  Handle GPIO EXTI interrupt (clear pending bit).
 *         Call this from the appropriate EXTIx_IRQHandler.
 * @param  pin: Pin number that triggered the interrupt
 */
void GPIO_IRQHandling(gpio_pin_t pin);

#ifdef __cplusplus
}
#endif

#endif /* GPIO_DRIVER_H */

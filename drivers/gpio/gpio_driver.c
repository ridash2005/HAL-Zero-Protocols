/**
 * @file    gpio_driver.c
 * @brief   STM32F4xx GPIO Driver — Implementation
 * @version 2.0.0
 */

#include "gpio_driver.h"
#include "../nvic/nvic_driver.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  Clock Control
 * ═══════════════════════════════════════════════════════════════════════════
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t en) {
  if (en == ENABLE) {
    if (pGPIOx == GPIOA) {
      GPIOA_PCLK_EN();
    } else if (pGPIOx == GPIOB) {
      GPIOB_PCLK_EN();
    } else if (pGPIOx == GPIOC) {
      GPIOC_PCLK_EN();
    } else if (pGPIOx == GPIOD) {
      GPIOD_PCLK_EN();
    } else if (pGPIOx == GPIOE) {
      GPIOE_PCLK_EN();
    } else if (pGPIOx == GPIOF) {
      GPIOF_PCLK_EN();
    } else if (pGPIOx == GPIOG) {
      GPIOG_PCLK_EN();
    } else if (pGPIOx == GPIOH) {
      GPIOH_PCLK_EN();
    }
  } else {
    if (pGPIOx == GPIOA) {
      GPIOA_PCLK_DI();
    } else if (pGPIOx == GPIOB) {
      GPIOB_PCLK_DI();
    } else if (pGPIOx == GPIOC) {
      GPIOC_PCLK_DI();
    } else if (pGPIOx == GPIOD) {
      GPIOD_PCLK_DI();
    } else if (pGPIOx == GPIOE) {
      GPIOE_PCLK_DI();
    }
    /* Note: GPIOF/G/H disable macros not defined in base header —
     * add them to stm32f4xx_base.h if these ports are used. */
  }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Initialization / De-initialization
 * ═══════════════════════════════════════════════════════════════════════════
 */

drv_status_t GPIO_Init(GPIO_Handle_t *pHandle) {
  if (pHandle == NULL) {
    return DRV_INVALID_PARAM;
  }

  uint32_t temp = 0;
  GPIO_RegDef_t *pGPIOx = pHandle->pGPIOx;
  uint8_t pin = pHandle->config.pin_number;

  /* 1. Enable the peripheral clock */
  GPIO_PeriClockControl(pGPIOx, ENABLE);

  /* 2. Configure the mode */
  if (pHandle->config.mode <= GPIO_MODE_ANALOG) {
    /* Non-interrupt mode */
    temp = pHandle->config.mode << (2 * pin);
    pGPIOx->MODER &= ~(0x3 << (2 * pin)); /* Clear bits */
    pGPIOx->MODER |= temp;
  } else {
    /* Interrupt mode — configure EXTI */
    /* Set pin as input first */
    pGPIOx->MODER &= ~(0x3 << (2 * pin));

    if (pHandle->config.mode == GPIO_MODE_IT_FT) {
      /* Configure falling edge trigger */
      EXTI->FTSR |= (1 << pin);
      EXTI->RTSR &= ~(1 << pin);
    } else if (pHandle->config.mode == GPIO_MODE_IT_RT) {
      /* Configure rising edge trigger */
      EXTI->RTSR |= (1 << pin);
      EXTI->FTSR &= ~(1 << pin);
    } else if (pHandle->config.mode == GPIO_MODE_IT_RFT) {
      /* Configure both rising and falling */
      EXTI->RTSR |= (1 << pin);
      EXTI->FTSR |= (1 << pin);
    }

    /* Configure SYSCFG EXTICR to route the GPIO port to EXTI line */
    SYSCFG_PCLK_EN();
    uint8_t idx = pin / 4;
    uint8_t section = pin % 4;
    uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOx);
    SYSCFG->EXTICR[idx] &= ~(0xF << (4 * section));
    SYSCFG->EXTICR[idx] |= (portcode << (4 * section));

    /* Unmask the EXTI line */
    EXTI->IMR |= (1 << pin);
  }

  /* 3. Configure speed */
  temp = pHandle->config.speed << (2 * pin);
  pGPIOx->OSPEEDR &= ~(0x3 << (2 * pin));
  pGPIOx->OSPEEDR |= temp;

  /* 4. Configure pull-up/pull-down */
  temp = pHandle->config.pull << (2 * pin);
  pGPIOx->PUPDR &= ~(0x3 << (2 * pin));
  pGPIOx->PUPDR |= temp;

  /* 5. Configure output type */
  pGPIOx->OTYPER &= ~(1 << pin);
  pGPIOx->OTYPER |= (pHandle->config.output_type << pin);

  /* 6. Configure alternate function */
  if (pHandle->config.mode == GPIO_MODE_ALTFN) {
    uint8_t idx = pin / 8;
    uint8_t pos = pin % 8;
    pGPIOx->AFR[idx] &= ~(0xF << (4 * pos));
    pGPIOx->AFR[idx] |= (pHandle->config.alt_function << (4 * pos));
  }

  return DRV_OK;
}

drv_status_t GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
  if (pGPIOx == GPIOA) {
    GPIOA_REG_RESET();
  } else if (pGPIOx == GPIOB) {
    GPIOB_REG_RESET();
  } else if (pGPIOx == GPIOC) {
    GPIOC_REG_RESET();
  } else if (pGPIOx == GPIOD) {
    GPIOD_REG_RESET();
  } else if (pGPIOx == GPIOE) {
    GPIOE_REG_RESET();
  } else {
    return DRV_INVALID_PARAM;
  }
  return DRV_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Read Operations
 * ═══════════════════════════════════════════════════════════════════════════
 */

uint8_t GPIO_ReadPin(GPIO_RegDef_t *pGPIOx, gpio_pin_t pin) {
  return (uint8_t)((pGPIOx->IDR >> pin) & 0x1);
}

uint16_t GPIO_ReadPort(GPIO_RegDef_t *pGPIOx) {
  return (uint16_t)(pGPIOx->IDR);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Write Operations
 * ═══════════════════════════════════════════════════════════════════════════
 */

void GPIO_WritePin(GPIO_RegDef_t *pGPIOx, gpio_pin_t pin, uint8_t value) {
  if (value == SET) {
    /* Atomic set via BSRR lower 16 bits */
    pGPIOx->BSRR = (1U << pin);
  } else {
    /* Atomic reset via BSRR upper 16 bits */
    pGPIOx->BSRR = (1U << (pin + 16));
  }
}

void GPIO_WritePort(GPIO_RegDef_t *pGPIOx, uint16_t value) {
  pGPIOx->ODR = value;
}

void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, gpio_pin_t pin) {
  pGPIOx->ODR ^= (1 << pin);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Interrupt Configuration
 * ═══════════════════════════════════════════════════════════════════════════
 */

void GPIO_IRQConfig(IRQn_t irq_number, uint8_t en) {
  /* Delegate to the centralized NVIC driver to avoid code duplication */
  if (en == ENABLE) {
    NVIC_EnableInterrupt(irq_number);
  } else {
    NVIC_DisableInterrupt(irq_number);
  }
}

void GPIO_IRQPriorityConfig(IRQn_t irq_number, uint8_t priority) {
  /* Delegate to the centralized NVIC driver */
  NVIC_SetPriority(irq_number, priority);
}

void GPIO_IRQHandling(gpio_pin_t pin) {
  /* Clear the EXTI pending register bit for this pin.
   * PR is a write-1-to-clear register — use direct assignment
   * to avoid accidentally clearing other pending bits via RMW. */
  if (EXTI->PR & (1 << pin)) {
    EXTI->PR = (1 << pin);
  }
}

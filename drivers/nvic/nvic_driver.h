/**
 * @file    nvic_driver.h
 * @brief   STM32F4xx NVIC Abstraction Layer — Public API
 * @version 2.0.0
 *
 * @details Provides a clean abstraction over the ARM Cortex-M4 Nested
 *          Vectored Interrupt Controller (NVIC). Demonstrates deep MCU
 *          knowledge of the interrupt subsystem.
 *
 * Features:
 *          - Enable / Disable individual interrupts
 *          - Set / Get interrupt priority (with group/sub-priority)
 *          - Priority grouping configuration
 *          - Pending flag management
 *          - Global interrupt control (__enable_irq / __disable_irq)
 */

#ifndef NVIC_DRIVER_H
#define NVIC_DRIVER_H

#include "../common/error_codes.h"
#include "../common/stm32f4xx_base.h"


#ifdef __cplusplus
extern "C" {
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 *  NVIC Register Access Macros (Cortex-M4)
 * ═══════════════════════════════════════════════════════════════════════════
 */

/* ISER: Interrupt Set-Enable Registers (0xE000E100 - 0xE000E11C) */
#define NVIC_ISER_BASE ((__vo uint32_t *)0xE000E100UL)

/* ICER: Interrupt Clear-Enable Registers (0xE000E180 - 0xE000E19C) */
#define NVIC_ICER_BASE ((__vo uint32_t *)0xE000E180UL)

/* ISPR: Interrupt Set-Pending Registers (0xE000E200 - 0xE000E21C) */
#define NVIC_ISPR_BASE ((__vo uint32_t *)0xE000E200UL)

/* ICPR: Interrupt Clear-Pending Registers (0xE000E280 - 0xE000E29C) */
#define NVIC_ICPR_BASE ((__vo uint32_t *)0xE000E280UL)

/* IABR: Interrupt Active Bit Registers (0xE000E300 - 0xE000E31C) */
#define NVIC_IABR_BASE ((__vo uint32_t *)0xE000E300UL)

/* IPR:  Interrupt Priority Registers (0xE000E400 - 0xE000E4EF) */
#define NVIC_IPR_REGS ((__vo uint8_t *)0xE000E400UL)

/* STIR: Software Trigger Interrupt Register */
#define NVIC_STIR ((__vo uint32_t *)0xE000EF00UL)

/* SCB AIRCR: Application Interrupt and Reset Control Register */
#define SCB_AIRCR ((__vo uint32_t *)0xE000ED0CUL)
#define SCB_AIRCR_VECTKEY 0x05FA0000UL
#define SCB_AIRCR_PRIGROUP_MASK (0x7 << 8)

/* ═══════════════════════════════════════════════════════════════════════════
 *  Priority Grouping
 * ═══════════════════════════════════════════════════════════════════════════
 */

/**
 * @brief Priority group configuration.
 *
 * The STM32F4 implements 4 priority bits (bits [7:4] of each IPR byte).
 * The PRIGROUP field in SCB->AIRCR determines how these 4 bits are split
 * between group (preemption) priority and sub-priority.
 *
 *  PRIGROUP | Group bits | Sub-priority bits | Groups | Sub-priorities
 *  ---------|------------|-------------------|--------|---------------
 *     3     |    4       |       0           |   16   |      1
 *     4     |    3       |       1           |    8   |      2
 *     5     |    2       |       2           |    4   |      4
 *     6     |    1       |       3           |    2   |      8
 *     7     |    0       |       4           |    1   |     16
 */
typedef enum {
  NVIC_PRIGROUP_4_0 = 3, /*!< 4 bits group, 0 bits sub (16 levels) */
  NVIC_PRIGROUP_3_1 = 4, /*!< 3 bits group, 1 bit sub              */
  NVIC_PRIGROUP_2_2 = 5, /*!< 2 bits group, 2 bits sub             */
  NVIC_PRIGROUP_1_3 = 6, /*!< 1 bit group,  3 bits sub             */
  NVIC_PRIGROUP_0_4 = 7  /*!< 0 bits group, 4 bits sub             */
} nvic_prigroup_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════════════
 */

/**
 * @brief  Enable a specific interrupt in the NVIC.
 * @param  irq: IRQ number (0-239 for Cortex-M4)
 */
void NVIC_EnableInterrupt(IRQn_t irq);

/**
 * @brief  Disable a specific interrupt in the NVIC.
 * @param  irq: IRQ number
 */
void NVIC_DisableInterrupt(IRQn_t irq);

/**
 * @brief  Set the priority for an interrupt.
 * @param  irq: IRQ number
 * @param  priority: Priority value (0-15 for STM32F4, 0 = highest)
 *
 * @note   The priority value is automatically shifted into the implemented
 *         bits (upper 4 bits of the 8-bit field on STM32F4).
 */
void NVIC_SetPriority(IRQn_t irq, uint8_t priority);

/**
 * @brief  Get the configured priority for an interrupt.
 * @param  irq: IRQ number
 * @retval Priority value (0-15)
 */
uint8_t NVIC_GetPriority(IRQn_t irq);

/**
 * @brief  Set the priority grouping (affects all interrupts).
 * @param  group: Priority group configuration
 */
void NVIC_SetPriorityGrouping(nvic_prigroup_t group);

/**
 * @brief  Encode group and sub-priority into a single priority value.
 * @param  group_priority: Group (preemption) priority
 * @param  sub_priority: Sub-priority within the group
 * @param  group_config: Current priority grouping configuration
 * @retval Encoded priority value suitable for NVIC_SetPriority()
 */
uint8_t NVIC_EncodePriority(uint8_t group_priority, uint8_t sub_priority,
                            nvic_prigroup_t group_config);

/**
 * @brief  Set an interrupt as pending (software trigger).
 * @param  irq: IRQ number
 */
void NVIC_SetPending(IRQn_t irq);

/**
 * @brief  Clear the pending flag for an interrupt.
 * @param  irq: IRQ number
 */
void NVIC_ClearPending(IRQn_t irq);

/**
 * @brief  Check if an interrupt is currently active (being serviced).
 * @param  irq: IRQ number
 * @retval 1 if active, 0 if not
 */
uint8_t NVIC_IsActive(IRQn_t irq);

/**
 * @brief  Check if an interrupt is pending.
 * @param  irq: IRQ number
 * @retval 1 if pending, 0 if not
 */
uint8_t NVIC_IsPending(IRQn_t irq);

/**
 * @brief  Trigger an interrupt via software (using STIR register).
 * @param  irq: IRQ number
 */
void NVIC_SoftwareTrigger(IRQn_t irq);

/**
 * @brief  Enable all interrupts globally (PRIMASK = 0).
 */
void NVIC_GlobalEnableIRQ(void);

/**
 * @brief  Disable all interrupts globally (PRIMASK = 1).
 */
void NVIC_GlobalDisableIRQ(void);

/**
 * @brief  Set the base priority masking register.
 *         Interrupts with priority >= basepri are masked.
 * @param  basepri: Priority threshold (0 disables masking)
 */
void NVIC_SetBasePriority(uint8_t basepri);

#ifdef __cplusplus
}
#endif

#endif /* NVIC_DRIVER_H */

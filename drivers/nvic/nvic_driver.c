/**
 * @file    nvic_driver.c
 * @brief   STM32F4xx NVIC Abstraction Layer — Implementation
 * @version 2.0.0
 *
 * @details Implements the NVIC API by directly accessing Cortex-M4 system
 *          control registers. This demonstrates deep understanding of the
 *          ARM interrupt architecture.
 */

#include "nvic_driver.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  Enable / Disable Interrupts
 * ═══════════════════════════════════════════════════════════════════════════
 */

void NVIC_EnableInterrupt(IRQn_t irq) {
  if ((int32_t)irq < 0)
    return; /* Cannot enable system exceptions via NVIC */

  uint8_t reg_idx = (uint8_t)irq / 32;
  uint8_t bit_pos = (uint8_t)irq % 32;

  /* ISER is a write-1-to-set register */
  NVIC_ISER_BASE[reg_idx] = (1UL << bit_pos);
}

void NVIC_DisableInterrupt(IRQn_t irq) {
  if ((int32_t)irq < 0)
    return;

  uint8_t reg_idx = (uint8_t)irq / 32;
  uint8_t bit_pos = (uint8_t)irq % 32;

  /* ICER is a write-1-to-clear register */
  NVIC_ICER_BASE[reg_idx] = (1UL << bit_pos);

  /* Data Synchronization Barrier to ensure the NVIC sees the disable */
  __asm volatile("dsb" ::: "memory");
  __asm volatile("isb" ::: "memory");
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Priority Management
 * ═══════════════════════════════════════════════════════════════════════════
 */

void NVIC_SetPriority(IRQn_t irq, uint8_t priority) {
  if ((int32_t)irq < 0) {
    /*
     * System exceptions (negative IRQ numbers) have their priority
     * in the SCB->SHPRx registers. This is a simplified implementation
     * covering the most common case (peripheral IRQs only).
     */
    return;
  }

  /*
   * Each IPR register byte holds the priority for one interrupt.
   * STM32F4 implements only the upper 4 bits (bits [7:4]).
   * We shift the user's 4-bit priority into the correct position.
   */
  NVIC_IPR_REGS[(uint8_t)irq] = (priority << (8 - NVIC_PRIO_BITS));
}

uint8_t NVIC_GetPriority(IRQn_t irq) {
  if ((int32_t)irq < 0)
    return 0xFF;

  return NVIC_IPR_REGS[(uint8_t)irq] >> (8 - NVIC_PRIO_BITS);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Priority Grouping
 * ═══════════════════════════════════════════════════════════════════════════
 */

void NVIC_SetPriorityGrouping(nvic_prigroup_t group) {
  uint32_t reg = *SCB_AIRCR;

  /* Must write VECTKEY (0x05FA) to access PRIGROUP */
  reg &= ~(0xFFFF0000UL | SCB_AIRCR_PRIGROUP_MASK);
  reg |= SCB_AIRCR_VECTKEY;
  reg |= ((uint32_t)group << 8);

  *SCB_AIRCR = reg;
}

uint8_t NVIC_EncodePriority(uint8_t group_priority, uint8_t sub_priority,
                            nvic_prigroup_t group_config) {
  /*
   * Given the PRIGROUP value, determine how many bits are used for
   * group vs sub-priority. The total is always NVIC_PRIO_BITS (4).
   */
  uint8_t group_bits;

  switch (group_config) {
  case NVIC_PRIGROUP_4_0:
    group_bits = 4;
    break;
  case NVIC_PRIGROUP_3_1:
    group_bits = 3;
    break;
  case NVIC_PRIGROUP_2_2:
    group_bits = 2;
    break;
  case NVIC_PRIGROUP_1_3:
    group_bits = 1;
    break;
  case NVIC_PRIGROUP_0_4:
    group_bits = 0;
    break;
  default:
    group_bits = 4;
    break;
  }

  uint8_t sub_bits = NVIC_PRIO_BITS - group_bits;

  /* Mask and combine */
  uint8_t group_mask = (1 << group_bits) - 1;
  uint8_t sub_mask = (1 << sub_bits) - 1;

  return ((group_priority & group_mask) << sub_bits) |
         (sub_priority & sub_mask);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Pending Flags
 * ═══════════════════════════════════════════════════════════════════════════
 */

void NVIC_SetPending(IRQn_t irq) {
  if ((int32_t)irq < 0)
    return;

  uint8_t reg_idx = (uint8_t)irq / 32;
  uint8_t bit_pos = (uint8_t)irq % 32;

  NVIC_ISPR_BASE[reg_idx] = (1UL << bit_pos);
}

void NVIC_ClearPending(IRQn_t irq) {
  if ((int32_t)irq < 0)
    return;

  uint8_t reg_idx = (uint8_t)irq / 32;
  uint8_t bit_pos = (uint8_t)irq % 32;

  NVIC_ICPR_BASE[reg_idx] = (1UL << bit_pos);
}

uint8_t NVIC_IsPending(IRQn_t irq) {
  if ((int32_t)irq < 0)
    return 0;

  uint8_t reg_idx = (uint8_t)irq / 32;
  uint8_t bit_pos = (uint8_t)irq % 32;

  return (NVIC_ISPR_BASE[reg_idx] & (1UL << bit_pos)) ? 1 : 0;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Active Status
 * ═══════════════════════════════════════════════════════════════════════════
 */

uint8_t NVIC_IsActive(IRQn_t irq) {
  if ((int32_t)irq < 0)
    return 0;

  uint8_t reg_idx = (uint8_t)irq / 32;
  uint8_t bit_pos = (uint8_t)irq % 32;

  return (NVIC_IABR_BASE[reg_idx] & (1UL << bit_pos)) ? 1 : 0;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Software Trigger
 * ═══════════════════════════════════════════════════════════════════════════
 */

void NVIC_SoftwareTrigger(IRQn_t irq) {
  if ((int32_t)irq < 0)
    return;

  /* STIR accepts the interrupt ID (0-239) in bits [8:0] */
  *NVIC_STIR = ((uint32_t)irq & 0x1FF);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Global Interrupt Control (PRIMASK / BASEPRI)
 * ═══════════════════════════════════════════════════════════════════════════
 */

void NVIC_GlobalEnableIRQ(void) { __asm volatile("cpsie i" ::: "memory"); }

void NVIC_GlobalDisableIRQ(void) { __asm volatile("cpsid i" ::: "memory"); }

void NVIC_SetBasePriority(uint8_t basepri) {
  /*
   * BASEPRI: When non-zero, masks all interrupts with priority >= BASEPRI.
   * Setting to 0 disables the masking.
   */
  uint32_t val = (uint32_t)(basepri << (8 - NVIC_PRIO_BITS));
  __asm volatile("msr basepri, %0" ::"r"(val) : "memory");
}

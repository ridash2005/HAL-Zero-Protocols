/**
 * @file    stm32f4xx_base.h
 * @brief   STM32F4xx Base Register Definitions and Macros
 * @version 2.0.0
 *
 * @details This header provides memory-mapped register definitions for the
 *          STM32F4xx family (Cortex-M4F). It follows ARM CMSIS conventions
 *          and serves as the foundation for all bare-metal drivers in this
 * suite.
 *
 *          Target: STM32F401xx / STM32F411xx / STM32F446xx
 *
 * @note    This file is intentionally self-contained — no external CMSIS or
 *          HAL headers are required. This enables learning and portability.
 */

#ifndef STM32F4XX_BASE_H
#define STM32F4XX_BASE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 *  Compiler Abstraction
 * ═══════════════════════════════════════════════════════════════════════════
 */
#define __vo volatile
#define __IO volatile
#define __I volatile const
#if defined(_MSC_VER)
#define __weak
#else
#define __weak __attribute__((weak))
#endif
#define UNUSED(x) ((void)(x))

/* ═══════════════════════════════════════════════════════════════════════════
 *  ARM Cortex-M4 System Addresses
 * ═══════════════════════════════════════════════════════════════════════════
 */
#define NVIC_BASE_ADDR 0xE000E100UL
#define NVIC_PRIO_BASE_ADDR 0xE000E400UL
#define SCB_BASE_ADDR 0xE000ED00UL
#define SYSTICK_BASE_ADDR 0xE000E010UL
#define NVIC_STIR_ADDR 0xE000EF00UL

/* NVIC ISER (Interrupt Set-Enable Registers) */
#define NVIC_ISER0 ((__vo uint32_t *)0xE000E100UL)
#define NVIC_ISER1 ((__vo uint32_t *)0xE000E104UL)
#define NVIC_ISER2 ((__vo uint32_t *)0xE000E108UL)
#define NVIC_ISER3 ((__vo uint32_t *)0xE000E10CUL)

/* NVIC ICER (Interrupt Clear-Enable Registers) */
#define NVIC_ICER0 ((__vo uint32_t *)0xE000E180UL)
#define NVIC_ICER1 ((__vo uint32_t *)0xE000E184UL)
#define NVIC_ICER2 ((__vo uint32_t *)0xE000E188UL)
#define NVIC_ICER3 ((__vo uint32_t *)0xE000E18CUL)

/* NVIC IPR (Interrupt Priority Registers) */
#define NVIC_IPR_BASE ((__vo uint32_t *)0xE000E400UL)

/* Number of priority bits implemented (STM32F4 = 4 bits) */
#define NVIC_PRIO_BITS 4

/* ═══════════════════════════════════════════════════════════════════════════
 *  Memory Map — Bus Domains
 * ═══════════════════════════════════════════════════════════════════════════
 */

/* Flash and SRAM base addresses */
#define FLASH_BASE_ADDR 0x08000000UL
#define SRAM1_BASE_ADDR 0x20000000UL
#define SRAM2_BASE_ADDR 0x2001C000UL
#define ROM_BASE_ADDR 0x1FFF0000UL /* System memory */

/* Bus base addresses */
#define PERIPH_BASE_ADDR 0x40000000UL
#define APB1_BASE_ADDR PERIPH_BASE_ADDR
#define APB2_BASE_ADDR 0x40010000UL
#define AHB1_BASE_ADDR 0x40020000UL
#define AHB2_BASE_ADDR 0x50000000UL

/* ═══════════════════════════════════════════════════════════════════════════
 *  AHB1 Peripherals
 * ═══════════════════════════════════════════════════════════════════════════
 */
#define GPIOA_BASE_ADDR (AHB1_BASE_ADDR + 0x0000UL)
#define GPIOB_BASE_ADDR (AHB1_BASE_ADDR + 0x0400UL)
#define GPIOC_BASE_ADDR (AHB1_BASE_ADDR + 0x0800UL)
#define GPIOD_BASE_ADDR (AHB1_BASE_ADDR + 0x0C00UL)
#define GPIOE_BASE_ADDR (AHB1_BASE_ADDR + 0x1000UL)
#define GPIOF_BASE_ADDR (AHB1_BASE_ADDR + 0x1400UL)
#define GPIOG_BASE_ADDR (AHB1_BASE_ADDR + 0x1800UL)
#define GPIOH_BASE_ADDR (AHB1_BASE_ADDR + 0x1C00UL)
#define RCC_BASE_ADDR (AHB1_BASE_ADDR + 0x3800UL)
#define DMA1_BASE_ADDR (AHB1_BASE_ADDR + 0x6000UL)
#define DMA2_BASE_ADDR (AHB1_BASE_ADDR + 0x6400UL)

/* ═══════════════════════════════════════════════════════════════════════════
 *  APB1 Peripherals
 * ═══════════════════════════════════════════════════════════════════════════
 */
#define TIM2_BASE_ADDR (APB1_BASE_ADDR + 0x0000UL)
#define TIM3_BASE_ADDR (APB1_BASE_ADDR + 0x0400UL)
#define TIM4_BASE_ADDR (APB1_BASE_ADDR + 0x0800UL)
#define TIM5_BASE_ADDR (APB1_BASE_ADDR + 0x0C00UL)
#define SPI2_BASE_ADDR (APB1_BASE_ADDR + 0x3800UL)
#define SPI3_BASE_ADDR (APB1_BASE_ADDR + 0x3C00UL)
#define USART2_BASE_ADDR (APB1_BASE_ADDR + 0x4400UL)
#define USART3_BASE_ADDR (APB1_BASE_ADDR + 0x4800UL)
#define UART4_BASE_ADDR (APB1_BASE_ADDR + 0x4C00UL)
#define UART5_BASE_ADDR (APB1_BASE_ADDR + 0x5000UL)
#define I2C1_BASE_ADDR (APB1_BASE_ADDR + 0x5400UL)
#define I2C2_BASE_ADDR (APB1_BASE_ADDR + 0x5800UL)
#define I2C3_BASE_ADDR (APB1_BASE_ADDR + 0x5C00UL)
#define CAN1_BASE_ADDR (APB1_BASE_ADDR + 0x6400UL)
#define CAN2_BASE_ADDR (APB1_BASE_ADDR + 0x6800UL)

/* ═══════════════════════════════════════════════════════════════════════════
 *  APB2 Peripherals
 * ═══════════════════════════════════════════════════════════════════════════
 */
#define TIM1_BASE_ADDR (APB2_BASE_ADDR + 0x0000UL)
#define USART1_BASE_ADDR (APB2_BASE_ADDR + 0x1000UL)
#define USART6_BASE_ADDR (APB2_BASE_ADDR + 0x1400UL)
#define ADC1_BASE_ADDR (APB2_BASE_ADDR + 0x2000UL)
#define ADC_COMMON_BASE_ADDR (APB2_BASE_ADDR + 0x2300UL)
#define SPI1_BASE_ADDR (APB2_BASE_ADDR + 0x3000UL)
#define SPI4_BASE_ADDR (APB2_BASE_ADDR + 0x3400UL)
#define SYSCFG_BASE_ADDR (APB2_BASE_ADDR + 0x3800UL)
#define EXTI_BASE_ADDR (APB2_BASE_ADDR + 0x3C00UL)
#define TIM9_BASE_ADDR (APB2_BASE_ADDR + 0x4000UL)
#define TIM10_BASE_ADDR (APB2_BASE_ADDR + 0x4400UL)
#define TIM11_BASE_ADDR (APB2_BASE_ADDR + 0x4800UL)

/* USB OTG */
#define USB_OTG_FS_BASE_ADDR 0x50000000UL

/* ═══════════════════════════════════════════════════════════════════════════
 *  Peripheral Register Structures
 * ═══════════════════════════════════════════════════════════════════════════
 */

/* ── GPIO ───────────────────────────────────────────────────────────────── */
typedef struct {
  __vo uint32_t MODER;   /*!< Mode register,                    offset: 0x00 */
  __vo uint32_t OTYPER;  /*!< Output type register,             offset: 0x04 */
  __vo uint32_t OSPEEDR; /*!< Output speed register,            offset: 0x08 */
  __vo uint32_t PUPDR;   /*!< Pull-up/pull-down register,       offset: 0x0C */
  __vo uint32_t IDR;     /*!< Input data register,              offset: 0x10 */
  __vo uint32_t ODR;     /*!< Output data register,             offset: 0x14 */
  __vo uint32_t BSRR;    /*!< Bit set/reset register,           offset: 0x18 */
  __vo uint32_t LCKR;    /*!< Configuration lock register,      offset: 0x1C */
  __vo uint32_t
      AFR[2]; /*!< Alternate function registers,     offset: 0x20-0x24 */
} GPIO_RegDef_t;

/* ── RCC ────────────────────────────────────────────────────────────────── */
typedef struct {
  __vo uint32_t CR;       /*!< Clock control register,           offset: 0x00 */
  __vo uint32_t PLLCFGR;  /*!< PLL configuration register,       offset: 0x04 */
  __vo uint32_t CFGR;     /*!< Clock configuration register,     offset: 0x08 */
  __vo uint32_t CIR;      /*!< Clock interrupt register,         offset: 0x0C */
  __vo uint32_t AHB1RSTR; /*!< AHB1 peripheral reset register,   offset: 0x10 */
  __vo uint32_t AHB2RSTR; /*!< AHB2 peripheral reset register,   offset: 0x14 */
  __vo uint32_t AHB3RSTR; /*!< AHB3 peripheral reset register,   offset: 0x18 */
  uint32_t RESERVED0;     /*!< Reserved,                         offset: 0x1C */
  __vo uint32_t APB1RSTR; /*!< APB1 peripheral reset register,   offset: 0x20 */
  __vo uint32_t APB2RSTR; /*!< APB2 peripheral reset register,   offset: 0x24 */
  uint32_t
      RESERVED1[2]; /*!< Reserved,                         offset: 0x28-0x2C */
  __vo uint32_t AHB1ENR; /*!< AHB1 peripheral clock enable,     offset: 0x30 */
  __vo uint32_t AHB2ENR; /*!< AHB2 peripheral clock enable,     offset: 0x34 */
  __vo uint32_t AHB3ENR; /*!< AHB3 peripheral clock enable,     offset: 0x38 */
  uint32_t RESERVED2;    /*!< Reserved,                         offset: 0x3C */
  __vo uint32_t APB1ENR; /*!< APB1 peripheral clock enable,     offset: 0x40 */
  __vo uint32_t APB2ENR; /*!< APB2 peripheral clock enable,     offset: 0x44 */
  uint32_t
      RESERVED3[2]; /*!< Reserved,                         offset: 0x48-0x4C */
  __vo uint32_t
      AHB1LPENR; /*!< AHB1 low-power clock enable,      offset: 0x50 */
  __vo uint32_t
      AHB2LPENR; /*!< AHB2 low-power clock enable,      offset: 0x54 */
  __vo uint32_t
      AHB3LPENR;      /*!< AHB3 low-power clock enable,      offset: 0x58 */
  uint32_t RESERVED4; /*!< Reserved,                         offset: 0x5C */
  __vo uint32_t
      APB1LPENR; /*!< APB1 low-power clock enable,      offset: 0x60 */
  __vo uint32_t
      APB2LPENR; /*!< APB2 low-power clock enable,      offset: 0x64 */
  uint32_t
      RESERVED5[2]; /*!< Reserved,                         offset: 0x68-0x6C */
  __vo uint32_t BDCR; /*!< Backup domain control register,   offset: 0x70 */
  __vo uint32_t CSR;  /*!< Clock control & status register,  offset: 0x74 */
  uint32_t
      RESERVED6[2]; /*!< Reserved,                         offset: 0x78-0x7C */
  __vo uint32_t SSCGR; /*!< Spread spectrum clock gen register,offset: 0x80 */
  __vo uint32_t
      PLLI2SCFGR; /*!< PLLI2S configuration register,    offset: 0x84 */
  __vo uint32_t
      PLLSAICFGR;        /*!< PLLSAI configuration register,    offset: 0x88 */
  __vo uint32_t DCKCFGR; /*!< Dedicated Clocks configuration,   offset: 0x8C */
} RCC_RegDef_t;

/* ── EXTI ───────────────────────────────────────────────────────────────── */
typedef struct {
  __vo uint32_t IMR;   /*!< Interrupt mask register,          offset: 0x00 */
  __vo uint32_t EMR;   /*!< Event mask register,              offset: 0x04 */
  __vo uint32_t RTSR;  /*!< Rising trigger selection,         offset: 0x08 */
  __vo uint32_t FTSR;  /*!< Falling trigger selection,        offset: 0x0C */
  __vo uint32_t SWIER; /*!< Software interrupt event register,offset: 0x10 */
  __vo uint32_t PR;    /*!< Pending register,                 offset: 0x14 */
} EXTI_RegDef_t;

/* ── SYSCFG ─────────────────────────────────────────────────────────────── */
typedef struct {
  __vo uint32_t MEMRMP; /*!< Memory remap register,            offset: 0x00 */
  __vo uint32_t PMC;    /*!< Peripheral mode configuration,    offset: 0x04 */
  __vo uint32_t
      EXTICR[4]; /*!< External interrupt configuration, offset: 0x08-0x14 */
  uint32_t
      RESERVED[2]; /*!< Reserved,                         offset: 0x18-0x1C */
  __vo uint32_t CMPCR; /*!< Compensation cell control,        offset: 0x20 */
} SYSCFG_RegDef_t;

/* ── USART ──────────────────────────────────────────────────────────────── */
typedef struct {
  __vo uint32_t SR;   /*!< Status register,                  offset: 0x00 */
  __vo uint32_t DR;   /*!< Data register,                    offset: 0x04 */
  __vo uint32_t BRR;  /*!< Baud rate register,               offset: 0x08 */
  __vo uint32_t CR1;  /*!< Control register 1,               offset: 0x0C */
  __vo uint32_t CR2;  /*!< Control register 2,               offset: 0x10 */
  __vo uint32_t CR3;  /*!< Control register 3,               offset: 0x14 */
  __vo uint32_t GTPR; /*!< Guard time and prescaler,         offset: 0x18 */
} USART_RegDef_t;

/* ── SPI ────────────────────────────────────────────────────────────────── */
typedef struct {
  __vo uint32_t CR1;     /*!< Control register 1,               offset: 0x00 */
  __vo uint32_t CR2;     /*!< Control register 2,               offset: 0x04 */
  __vo uint32_t SR;      /*!< Status register,                  offset: 0x08 */
  __vo uint32_t DR;      /*!< Data register,                    offset: 0x0C */
  __vo uint32_t CRCPR;   /*!< CRC polynomial register,          offset: 0x10 */
  __vo uint32_t RXCRCR;  /*!< RX CRC register,                  offset: 0x14 */
  __vo uint32_t TXCRCR;  /*!< TX CRC register,                  offset: 0x18 */
  __vo uint32_t I2SCFGR; /*!< I2S configuration register,       offset: 0x1C */
  __vo uint32_t I2SPR;   /*!< I2S prescaler register,           offset: 0x20 */
} SPI_RegDef_t;

/* ── I2C ────────────────────────────────────────────────────────────────── */
typedef struct {
  __vo uint32_t CR1;   /*!< Control register 1,               offset: 0x00 */
  __vo uint32_t CR2;   /*!< Control register 2,               offset: 0x04 */
  __vo uint32_t OAR1;  /*!< Own address register 1,           offset: 0x08 */
  __vo uint32_t OAR2;  /*!< Own address register 2,           offset: 0x0C */
  __vo uint32_t DR;    /*!< Data register,                    offset: 0x10 */
  __vo uint32_t SR1;   /*!< Status register 1,                offset: 0x14 */
  __vo uint32_t SR2;   /*!< Status register 2,                offset: 0x18 */
  __vo uint32_t CCR;   /*!< Clock control register,           offset: 0x1C */
  __vo uint32_t TRISE; /*!< TRISE register,                   offset: 0x20 */
  __vo uint32_t FLTR;  /*!< FLTR register,                    offset: 0x24 */
} I2C_RegDef_t;

/* ── Timer (General Purpose: TIM2-TIM5) ─────────────────────────────────── */
typedef struct {
  __vo uint32_t CR1;   /*!< Control register 1,               offset: 0x00 */
  __vo uint32_t CR2;   /*!< Control register 2,               offset: 0x04 */
  __vo uint32_t SMCR;  /*!< Slave mode control register,      offset: 0x08 */
  __vo uint32_t DIER;  /*!< DMA/Interrupt enable register,    offset: 0x0C */
  __vo uint32_t SR;    /*!< Status register,                  offset: 0x10 */
  __vo uint32_t EGR;   /*!< Event generation register,        offset: 0x14 */
  __vo uint32_t CCMR1; /*!< Capture/Compare mode register 1,  offset: 0x18 */
  __vo uint32_t CCMR2; /*!< Capture/Compare mode register 2,  offset: 0x1C */
  __vo uint32_t CCER;  /*!< Capture/Compare enable register,  offset: 0x20 */
  __vo uint32_t CNT;   /*!< Counter,                          offset: 0x24 */
  __vo uint32_t PSC;   /*!< Prescaler,                        offset: 0x28 */
  __vo uint32_t ARR;   /*!< Auto-reload register,             offset: 0x2C */
  __vo uint32_t RCR;   /*!< Repetition counter register,      offset: 0x30 */
  __vo uint32_t CCR1;  /*!< Capture/Compare register 1,       offset: 0x34 */
  __vo uint32_t CCR2;  /*!< Capture/Compare register 2,       offset: 0x38 */
  __vo uint32_t CCR3;  /*!< Capture/Compare register 3,       offset: 0x3C */
  __vo uint32_t CCR4;  /*!< Capture/Compare register 4,       offset: 0x40 */
  __vo uint32_t BDTR;  /*!< Break and dead-time register,     offset: 0x44 */
  __vo uint32_t DCR;   /*!< DMA control register,             offset: 0x48 */
  __vo uint32_t DMAR;  /*!< DMA address for full transfer,    offset: 0x4C */
} TIM_RegDef_t;

/* ── ADC ────────────────────────────────────────────────────────────────── */
typedef struct {
  __vo uint32_t SR;    /*!< Status register,                  offset: 0x00 */
  __vo uint32_t CR1;   /*!< Control register 1,               offset: 0x04 */
  __vo uint32_t CR2;   /*!< Control register 2,               offset: 0x08 */
  __vo uint32_t SMPR1; /*!< Sample time register 1,           offset: 0x0C */
  __vo uint32_t SMPR2; /*!< Sample time register 2,           offset: 0x10 */
  __vo uint32_t JOFR1; /*!< Injected channel data offset 1,   offset: 0x14 */
  __vo uint32_t JOFR2; /*!< Injected channel data offset 2,   offset: 0x18 */
  __vo uint32_t JOFR3; /*!< Injected channel data offset 3,   offset: 0x1C */
  __vo uint32_t JOFR4; /*!< Injected channel data offset 4,   offset: 0x20 */
  __vo uint32_t HTR;   /*!< Watchdog higher threshold,        offset: 0x24 */
  __vo uint32_t LTR;   /*!< Watchdog lower threshold,         offset: 0x28 */
  __vo uint32_t SQR1;  /*!< Regular sequence register 1,      offset: 0x2C */
  __vo uint32_t SQR2;  /*!< Regular sequence register 2,      offset: 0x30 */
  __vo uint32_t SQR3;  /*!< Regular sequence register 3,      offset: 0x34 */
  __vo uint32_t JSQR;  /*!< Injected sequence register,       offset: 0x38 */
  __vo uint32_t JDR1;  /*!< Injected data register 1,         offset: 0x3C */
  __vo uint32_t JDR2;  /*!< Injected data register 2,         offset: 0x40 */
  __vo uint32_t JDR3;  /*!< Injected data register 3,         offset: 0x44 */
  __vo uint32_t JDR4;  /*!< Injected data register 4,         offset: 0x48 */
  __vo uint32_t DR;    /*!< Regular data register,            offset: 0x4C */
} ADC_RegDef_t;

/* ── ADC Common ─────────────────────────────────────────────────────────── */
typedef struct {
  __vo uint32_t CSR; /*!< Common status register,           offset: 0x00 */
  __vo uint32_t CCR; /*!< Common control register,          offset: 0x04 */
  __vo uint32_t CDR; /*!< Common regular data register,     offset: 0x08 */
} ADC_Common_RegDef_t;

/* ── DMA Stream ─────────────────────────────────────────────────────────── */
typedef struct {
  __vo uint32_t CR;   /*!< Stream configuration register,    offset: 0x00 */
  __vo uint32_t NDTR; /*!< Number of data to transfer,       offset: 0x04 */
  __vo uint32_t PAR;  /*!< Peripheral address register,      offset: 0x08 */
  __vo uint32_t M0AR; /*!< Memory 0 address register,        offset: 0x0C */
  __vo uint32_t M1AR; /*!< Memory 1 address register,        offset: 0x10 */
  __vo uint32_t FCR;  /*!< FIFO control register,            offset: 0x14 */
} DMA_Stream_RegDef_t;

/* ── DMA Controller ─────────────────────────────────────────────────────── */
typedef struct {
  __vo uint32_t LISR;  /*!< Low interrupt status register,    offset: 0x00 */
  __vo uint32_t HISR;  /*!< High interrupt status register,   offset: 0x04 */
  __vo uint32_t LIFCR; /*!< Low interrupt flag clear register,offset: 0x08 */
  __vo uint32_t HIFCR; /*!< High interrupt flag clear register,offset: 0x0C */
  DMA_Stream_RegDef_t
      STREAM[8]; /*!< Stream 0-7,                   offset: 0x10+ */
} DMA_RegDef_t;

/* ── CAN Mailbox ────────────────────────────────────────────────────────── */
typedef struct {
  __vo uint32_t TIR;  /*!< TX mailbox identifier register    */
  __vo uint32_t TDTR; /*!< TX mailbox data length & timestamp*/
  __vo uint32_t TDLR; /*!< TX mailbox data low register      */
  __vo uint32_t TDHR; /*!< TX mailbox data high register     */
} CAN_TxMailbox_t;

typedef struct {
  __vo uint32_t RIR;  /*!< RX FIFO mailbox identifier register */
  __vo uint32_t RDTR; /*!< RX FIFO mailbox data length & timestamp */
  __vo uint32_t RDLR; /*!< RX FIFO mailbox data low register */
  __vo uint32_t RDHR; /*!< RX FIFO mailbox data high register */
} CAN_RxMailbox_t;

typedef struct {
  __vo uint32_t FR1; /*!< Filter bank register 1            */
  __vo uint32_t FR2; /*!< Filter bank register 2            */
} CAN_FilterBank_t;

/* ── CAN Controller ─────────────────────────────────────────────────────── */
typedef struct {
  __vo uint32_t MCR;  /*!< Master control register,          offset: 0x000 */
  __vo uint32_t MSR;  /*!< Master status register,           offset: 0x004 */
  __vo uint32_t TSR;  /*!< Transmit status register,         offset: 0x008 */
  __vo uint32_t RF0R; /*!< Receive FIFO 0 register,          offset: 0x00C */
  __vo uint32_t RF1R; /*!< Receive FIFO 1 register,          offset: 0x010 */
  __vo uint32_t IER;  /*!< Interrupt enable register,        offset: 0x014 */
  __vo uint32_t ESR;  /*!< Error status register,            offset: 0x018 */
  __vo uint32_t BTR;  /*!< Bit timing register,              offset: 0x01C */
  uint32_t RESERVED0[88]; /*!< Reserved,                         offset:
                             0x020-0x17F */
  CAN_TxMailbox_t
      TX_MB[3]; /*!< TX Mailboxes 0-2,                 offset: 0x180 */
  CAN_RxMailbox_t
      RX_FIFO[2]; /*!< RX FIFOs 0-1,                     offset: 0x1B0 */
  uint32_t RESERVED1[12]; /*!< Reserved,                         offset:
                             0x1D0-0x1FF */
  __vo uint32_t FMR;     /*!< Filter master register,           offset: 0x200 */
  __vo uint32_t FM1R;    /*!< Filter mode register,             offset: 0x204 */
  uint32_t RESERVED2;    /*!< Reserved,                         offset: 0x208 */
  __vo uint32_t FS1R;    /*!< Filter scale register,            offset: 0x20C */
  uint32_t RESERVED3;    /*!< Reserved,                         offset: 0x210 */
  __vo uint32_t FFA1R;   /*!< Filter FIFO assignment register,  offset: 0x214 */
  uint32_t RESERVED4;    /*!< Reserved,                         offset: 0x218 */
  __vo uint32_t FA1R;    /*!< Filter activation register,       offset: 0x21C */
  uint32_t RESERVED5[8]; /*!< Reserved,                         offset:
                            0x220-0x23F */
  CAN_FilterBank_t
      FILTER[28]; /*!< Filter banks 0-27,                offset: 0x240 */
} CAN_RegDef_t;

/* ── USB OTG ────────────────────────────────────────────────────────────── */
/* USB OTG FS Global registers */
typedef struct {
  __vo uint32_t GOTGCTL; /*!< OTG control and status,           offset: 0x000 */
  __vo uint32_t GOTGINT; /*!< OTG interrupt,                    offset: 0x004 */
  __vo uint32_t GAHBCFG; /*!< AHB configuration,                offset: 0x008 */
  __vo uint32_t GUSBCFG; /*!< USB configuration,                offset: 0x00C */
  __vo uint32_t GRSTCTL; /*!< Reset register,                   offset: 0x010 */
  __vo uint32_t GINTSTS; /*!< Core interrupt register,          offset: 0x014 */
  __vo uint32_t GINTMSK; /*!< Interrupt mask register,          offset: 0x018 */
  __vo uint32_t GRXSTSR; /*!< Receive status debug read,        offset: 0x01C */
  __vo uint32_t GRXSTSP; /*!< Receive status read and pop,      offset: 0x020 */
  __vo uint32_t GRXFSIZ; /*!< Receive FIFO size register,       offset: 0x024 */
  __vo uint32_t
      DIEPTXF0; /*!< EP0 TX FIFO size,                 offset: 0x028 */
  __vo uint32_t
      HNPTXSTS;          /*!< Non-periodic TX FIFO/queue status,offset: 0x02C */
  uint32_t RESERVED0[2]; /*!< Reserved,                         offset:
                            0x030-0x034 */
  __vo uint32_t GCCFG;   /*!< General core configuration,       offset: 0x038 */
  __vo uint32_t CID;     /*!< Core ID register,                 offset: 0x03C */
  uint32_t RESERVED1[48]; /*!< Reserved,                         offset:
                             0x040-0x0FF */
  __vo uint32_t
      HPTXFSIZ; /*!< Host periodic TX FIFO size,       offset: 0x100 */
  __vo uint32_t
      DIEPTXF[3]; /*!< Device IN EP TX FIFO size 1-3,    offset: 0x104-0x10C */
} USB_OTG_Global_RegDef_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Peripheral Pointer Macros
 * ═══════════════════════════════════════════════════════════════════════════
 */
#define GPIOA ((GPIO_RegDef_t *)GPIOA_BASE_ADDR)
#define GPIOB ((GPIO_RegDef_t *)GPIOB_BASE_ADDR)
#define GPIOC ((GPIO_RegDef_t *)GPIOC_BASE_ADDR)
#define GPIOD ((GPIO_RegDef_t *)GPIOD_BASE_ADDR)
#define GPIOE ((GPIO_RegDef_t *)GPIOE_BASE_ADDR)
#define GPIOF ((GPIO_RegDef_t *)GPIOF_BASE_ADDR)
#define GPIOG ((GPIO_RegDef_t *)GPIOG_BASE_ADDR)
#define GPIOH ((GPIO_RegDef_t *)GPIOH_BASE_ADDR)

#define RCC ((RCC_RegDef_t *)RCC_BASE_ADDR)

#define EXTI ((EXTI_RegDef_t *)EXTI_BASE_ADDR)
#define SYSCFG ((SYSCFG_RegDef_t *)SYSCFG_BASE_ADDR)

#define USART1 ((USART_RegDef_t *)USART1_BASE_ADDR)
#define USART2 ((USART_RegDef_t *)USART2_BASE_ADDR)
#define USART3 ((USART_RegDef_t *)USART3_BASE_ADDR)
#define UART4 ((USART_RegDef_t *)UART4_BASE_ADDR)
#define UART5 ((USART_RegDef_t *)UART5_BASE_ADDR)
#define USART6 ((USART_RegDef_t *)USART6_BASE_ADDR)

#define SPI1 ((SPI_RegDef_t *)SPI1_BASE_ADDR)
#define SPI2 ((SPI_RegDef_t *)SPI2_BASE_ADDR)
#define SPI3 ((SPI_RegDef_t *)SPI3_BASE_ADDR)
#define SPI4 ((SPI_RegDef_t *)SPI4_BASE_ADDR)

#define I2C1 ((I2C_RegDef_t *)I2C1_BASE_ADDR)
#define I2C2 ((I2C_RegDef_t *)I2C2_BASE_ADDR)
#define I2C3 ((I2C_RegDef_t *)I2C3_BASE_ADDR)

#define TIM1 ((TIM_RegDef_t *)TIM1_BASE_ADDR)
#define TIM2 ((TIM_RegDef_t *)TIM2_BASE_ADDR)
#define TIM3 ((TIM_RegDef_t *)TIM3_BASE_ADDR)
#define TIM4 ((TIM_RegDef_t *)TIM4_BASE_ADDR)
#define TIM5 ((TIM_RegDef_t *)TIM5_BASE_ADDR)
#define TIM9 ((TIM_RegDef_t *)TIM9_BASE_ADDR)
#define TIM10 ((TIM_RegDef_t *)TIM10_BASE_ADDR)
#define TIM11 ((TIM_RegDef_t *)TIM11_BASE_ADDR)

#define ADC1 ((ADC_RegDef_t *)ADC1_BASE_ADDR)
#define ADC_COMMON ((ADC_Common_RegDef_t *)ADC_COMMON_BASE_ADDR)

#define DMA1 ((DMA_RegDef_t *)DMA1_BASE_ADDR)
#define DMA2 ((DMA_RegDef_t *)DMA2_BASE_ADDR)

#define CAN1 ((CAN_RegDef_t *)CAN1_BASE_ADDR)
#define CAN2 ((CAN_RegDef_t *)CAN2_BASE_ADDR)

#define USB_OTG_FS ((USB_OTG_Global_RegDef_t *)USB_OTG_FS_BASE_ADDR)

/* ═══════════════════════════════════════════════════════════════════════════
 *  Clock Enable Macros
 * ═══════════════════════════════════════════════════════════════════════════
 */

/* GPIO Clock Enable */
#define GPIOA_PCLK_EN() (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN() (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN() (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN() (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN() (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN() (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN() (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN() (RCC->AHB1ENR |= (1 << 7))

/* GPIO Clock Disable */
#define GPIOA_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 7))

/* SPI Clock Enable */
#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN() (RCC->APB2ENR |= (1 << 13))

/* SPI Clock Disable */
#define SPI1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI() (RCC->APB2ENR &= ~(1 << 13))

/* USART Clock Enable */
#define USART1_PCLK_EN() (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN() (RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN() (RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN() (RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN() (RCC->APB2ENR |= (1 << 5))

/* USART Clock Disable */
#define USART1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI() (RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI() (RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI() (RCC->APB2ENR &= ~(1 << 5))

/* I2C Clock Enable */
#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23))

/* I2C Clock Disable */
#define I2C1_PCLK_DI() (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 23))

/* Timer Clock Enable */
#define TIM1_PCLK_EN() (RCC->APB2ENR |= (1 << 0))
#define TIM2_PCLK_EN() (RCC->APB1ENR |= (1 << 0))
#define TIM3_PCLK_EN() (RCC->APB1ENR |= (1 << 1))
#define TIM4_PCLK_EN() (RCC->APB1ENR |= (1 << 2))
#define TIM5_PCLK_EN() (RCC->APB1ENR |= (1 << 3))

/* Timer Clock Disable */
#define TIM1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 0))
#define TIM2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 0))
#define TIM3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 1))
#define TIM4_PCLK_DI() (RCC->APB1ENR &= ~(1 << 2))
#define TIM5_PCLK_DI() (RCC->APB1ENR &= ~(1 << 3))

/* ADC Clock Enable */
#define ADC1_PCLK_EN() (RCC->APB2ENR |= (1 << 8))
#define ADC1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 8))

/* DMA Clock Enable */
#define DMA1_PCLK_EN() (RCC->AHB1ENR |= (1 << 21))
#define DMA2_PCLK_EN() (RCC->AHB1ENR |= (1 << 22))
#define DMA1_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 21))
#define DMA2_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 22))

/* SYSCFG Clock Enable */
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))
#define SYSCFG_PCLK_DI() (RCC->APB2ENR &= ~(1 << 14))

/* CAN Clock Enable */
#define CAN1_PCLK_EN() (RCC->APB1ENR |= (1 << 25))
#define CAN2_PCLK_EN() (RCC->APB1ENR |= (1 << 26))
#define CAN1_PCLK_DI() (RCC->APB1ENR &= ~(1 << 25))
#define CAN2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 26))

/* USB OTG Clock Enable */
#define USB_OTG_FS_PCLK_EN() (RCC->AHB2ENR |= (1 << 7))
#define USB_OTG_FS_PCLK_DI() (RCC->AHB2ENR &= ~(1 << 7))

/* ═══════════════════════════════════════════════════════════════════════════
 *  GPIO Reset Macros
 * ═══════════════════════════════════════════════════════════════════════════
 */
#define GPIOA_REG_RESET()                                                      \
  do {                                                                         \
    RCC->AHB1RSTR |= (1 << 0);                                                 \
    RCC->AHB1RSTR &= ~(1 << 0);                                                \
  } while (0)
#define GPIOB_REG_RESET()                                                      \
  do {                                                                         \
    RCC->AHB1RSTR |= (1 << 1);                                                 \
    RCC->AHB1RSTR &= ~(1 << 1);                                                \
  } while (0)
#define GPIOC_REG_RESET()                                                      \
  do {                                                                         \
    RCC->AHB1RSTR |= (1 << 2);                                                 \
    RCC->AHB1RSTR &= ~(1 << 2);                                                \
  } while (0)
#define GPIOD_REG_RESET()                                                      \
  do {                                                                         \
    RCC->AHB1RSTR |= (1 << 3);                                                 \
    RCC->AHB1RSTR &= ~(1 << 3);                                                \
  } while (0)
#define GPIOE_REG_RESET()                                                      \
  do {                                                                         \
    RCC->AHB1RSTR |= (1 << 4);                                                 \
    RCC->AHB1RSTR &= ~(1 << 4);                                                \
  } while (0)

/* SPI Reset */
#define SPI1_REG_RESET()                                                       \
  do {                                                                         \
    RCC->APB2RSTR |= (1 << 12);                                                \
    RCC->APB2RSTR &= ~(1 << 12);                                               \
  } while (0)
#define SPI2_REG_RESET()                                                       \
  do {                                                                         \
    RCC->APB1RSTR |= (1 << 14);                                                \
    RCC->APB1RSTR &= ~(1 << 14);                                               \
  } while (0)
#define SPI3_REG_RESET()                                                       \
  do {                                                                         \
    RCC->APB1RSTR |= (1 << 15);                                                \
    RCC->APB1RSTR &= ~(1 << 15);                                               \
  } while (0)

/* I2C Reset */
#define I2C1_REG_RESET()                                                       \
  do {                                                                         \
    RCC->APB1RSTR |= (1 << 21);                                                \
    RCC->APB1RSTR &= ~(1 << 21);                                               \
  } while (0)
#define I2C2_REG_RESET()                                                       \
  do {                                                                         \
    RCC->APB1RSTR |= (1 << 22);                                                \
    RCC->APB1RSTR &= ~(1 << 22);                                               \
  } while (0)
#define I2C3_REG_RESET()                                                       \
  do {                                                                         \
    RCC->APB1RSTR |= (1 << 23);                                                \
    RCC->APB1RSTR &= ~(1 << 23);                                               \
  } while (0)

/* USART Reset */
#define USART1_REG_RESET()                                                     \
  do {                                                                         \
    RCC->APB2RSTR |= (1 << 4);                                                 \
    RCC->APB2RSTR &= ~(1 << 4);                                                \
  } while (0)
#define USART2_REG_RESET()                                                     \
  do {                                                                         \
    RCC->APB1RSTR |= (1 << 17);                                                \
    RCC->APB1RSTR &= ~(1 << 17);                                               \
  } while (0)

/* ═══════════════════════════════════════════════════════════════════════════
 *  GPIO Port-to-Code Conversion (for EXTI configuration)
 * ═══════════════════════════════════════════════════════════════════════════
 */
#define GPIO_BASEADDR_TO_CODE(x)                                               \
  ((x == GPIOA)   ? 0                                                          \
   : (x == GPIOB) ? 1                                                          \
   : (x == GPIOC) ? 2                                                          \
   : (x == GPIOD) ? 3                                                          \
   : (x == GPIOE) ? 4                                                          \
   : (x == GPIOF) ? 5                                                          \
   : (x == GPIOG) ? 6                                                          \
   : (x == GPIOH) ? 7                                                          \
                  : 0)

/* ═══════════════════════════════════════════════════════════════════════════
 *  IRQ Numbers (STM32F4xx)
 * ═══════════════════════════════════════════════════════════════════════════
 */
typedef enum {
  /* Cortex-M4 System Exceptions (negative numbers) */
  IRQ_NMI = -14,
  IRQ_HARDFAULT = -13,
  IRQ_MEMMANAGE = -12,
  IRQ_BUSFAULT = -11,
  IRQ_USAGEFAULT = -10,
  IRQ_SVCALL = -5,
  IRQ_DEBUGMON = -4,
  IRQ_PENDSV = -2,
  IRQ_SYSTICK = -1,

  /* STM32F4xx Peripheral Interrupts */
  IRQ_WWDG = 0,
  IRQ_PVD = 1,
  IRQ_TAMP_STAMP = 2,
  IRQ_RTC_WKUP = 3,
  IRQ_FLASH = 4,
  IRQ_RCC = 5,
  IRQ_EXTI0 = 6,
  IRQ_EXTI1 = 7,
  IRQ_EXTI2 = 8,
  IRQ_EXTI3 = 9,
  IRQ_EXTI4 = 10,
  IRQ_DMA1_STREAM0 = 11,
  IRQ_DMA1_STREAM1 = 12,
  IRQ_DMA1_STREAM2 = 13,
  IRQ_DMA1_STREAM3 = 14,
  IRQ_DMA1_STREAM4 = 15,
  IRQ_DMA1_STREAM5 = 16,
  IRQ_DMA1_STREAM6 = 17,
  IRQ_ADC = 18,
  IRQ_CAN1_TX = 19,
  IRQ_CAN1_RX0 = 20,
  IRQ_CAN1_RX1 = 21,
  IRQ_CAN1_SCE = 22,
  IRQ_EXTI9_5 = 23,
  IRQ_TIM1_BRK_TIM9 = 24,
  IRQ_TIM1_UP_TIM10 = 25,
  IRQ_TIM1_TRG_COM_TIM11 = 26,
  IRQ_TIM1_CC = 27,
  IRQ_TIM2 = 28,
  IRQ_TIM3 = 29,
  IRQ_TIM4 = 30,
  IRQ_I2C1_EV = 31,
  IRQ_I2C1_ER = 32,
  IRQ_I2C2_EV = 33,
  IRQ_I2C2_ER = 34,
  IRQ_SPI1 = 35,
  IRQ_SPI2 = 36,
  IRQ_USART1 = 37,
  IRQ_USART2 = 38,
  IRQ_USART3 = 39,
  IRQ_EXTI15_10 = 40,
  IRQ_RTC_ALARM = 41,
  IRQ_OTG_FS_WKUP = 42,
  IRQ_DMA1_STREAM7 = 47,
  IRQ_TIM5 = 50,
  IRQ_SPI3 = 51,
  IRQ_UART4 = 52,
  IRQ_UART5 = 53,
  IRQ_DMA2_STREAM0 = 56,
  IRQ_DMA2_STREAM1 = 57,
  IRQ_DMA2_STREAM2 = 58,
  IRQ_DMA2_STREAM3 = 59,
  IRQ_DMA2_STREAM4 = 60,
  IRQ_OTG_FS = 67,
  IRQ_DMA2_STREAM5 = 68,
  IRQ_DMA2_STREAM6 = 69,
  IRQ_DMA2_STREAM7 = 70,
  IRQ_USART6 = 71,
  IRQ_I2C3_EV = 72,
  IRQ_I2C3_ER = 73,
  IRQ_SPI4 = 84,
} IRQn_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Bit Manipulation Helpers
 * ═══════════════════════════════════════════════════════════════════════════
 */
#define SET_BIT(REG, BIT) ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT) ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT) ((REG) & (BIT))
#define MODIFY_REG(REG, CLEARMASK, SETMASK)                                    \
  ((REG) = (((REG) & (~(CLEARMASK))) | (SETMASK)))

/* Generic flag operations */
#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE
#define FLAG_SET SET
#define FLAG_RESET RESET

/* Default system clock (HSI) */
#ifndef SYSTEM_CLOCK_HZ
#define SYSTEM_CLOCK_HZ 16000000UL /* 16 MHz HSI default */
#endif

#ifdef __cplusplus
}
#endif

#endif /* STM32F4XX_BASE_H */

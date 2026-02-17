# ⚙️ STM32F4xx Peripheral Driver Suite

> **Production-grade, bare-metal peripheral drivers** for the STM32F4xx family (Cortex-M4F).  
> Every driver is written from scratch with direct register manipulation — zero HAL dependencies.

---

## 📂 Directory Structure

```
drivers/
├── README.md               ← You are here
├── common/
│   ├── stm32f4xx_base.h    ← Memory map, register structures, clock macros
│   └── error_codes.h       ← Unified status codes & callback events
│
├── gpio/
│   ├── gpio_driver.h       ← Pin configuration, EXTI interrupt support
│   └── gpio_driver.c
│
├── uart/
│   ├── uart_driver.h       ← Polling, Interrupt, and DMA transfer modes
│   └── uart_driver.c
│
├── spi/
│   ├── spi_driver.h        ← Master/Slave, full/half-duplex, interrupt TX/RX
│   └── spi_driver.c
│
├── i2c/
│   ├── i2c_driver.h        ← Standard/Fast mode, blocking + interrupt master TX/RX
│   └── i2c_driver.c
│
├── timer/
│   ├── timer_driver.h      ← Up/Down counting, one-pulse, update interrupt, delays
│   └── timer_driver.c
│
├── adc/
│   ├── adc_driver.h        ← Single/continuous conversion, scan, DMA, temp sensor
│   └── adc_driver.c
│
├── pwm/
│   ├── pwm_driver.h        ← Frequency/duty control, 4 channels, runtime adjustment
│   └── pwm_driver.c
│
├── dma/
│   ├── dma_driver.h        ← Mem↔Periph, Mem↔Mem, circular, FIFO, burst
│   └── dma_driver.c
│
└── nvic/
    ├── nvic_driver.h        ← Enable/Disable IRQ, priority grouping, PRIMASK/BASEPRI
    └── nvic_driver.c
```

---

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────┐
│                 Application Code                     │
├─────────────────────────────────────────────────────┤
│  GPIO │ UART │ SPI │ I2C │ TIM │ ADC │ PWM │ DMA   │
├─────────────────────────────────────────────────────┤
│        NVIC Abstraction Layer (nvic_driver)          │
├─────────────────────────────────────────────────────┤
│  stm32f4xx_base.h  │  error_codes.h                 │
├─────────────────────────────────────────────────────┤
│            STM32F4xx Hardware (Cortex-M4F)            │
└─────────────────────────────────────────────────────┘
```

---

## ✅ Peripheral Status

| Driver  | Polling | Interrupt | DMA | Status |
|---------|:-------:|:---------:|:---:|--------|
| **GPIO**  | ✅ | ✅ (EXTI) | — | Complete |
| **UART**  | ✅ | ✅ | ✅ | Complete |
| **SPI**   | ✅ | ✅ | — | Complete |
| **I2C**   | ✅ | ✅ | — | Complete |
| **Timer** | ✅ | ✅ | — | Complete |
| **ADC**   | ✅ | ✅ | ✅ | Complete |
| **PWM**   | ✅ | — | — | Complete |
| **DMA**   | ✅ | ✅ | N/A | Complete |
| **NVIC**  | N/A | N/A | N/A | Complete |

---

## 🎯 Design Principles

1. **Zero Dependencies** — No HAL, no CMSIS headers, no vendor SDK
2. **Direct Register Access** — Every bit is manipulated manually for full transparency
3. **Consistent API** — All drivers share the same Init/DeInit/Control pattern
4. **Callback-Driven** — Non-blocking operations use function pointers for async notification
5. **Error Handling** — Every function returns `drv_status_t` for robust error propagation
6. **Documented** — Doxygen-compatible comments on every public function

---

## 🚀 Quick Start

```c
#include "drivers/common/stm32f4xx_base.h"
#include "drivers/gpio/gpio_driver.h"
#include "drivers/uart/uart_driver.h"

int main(void)
{
    /* === GPIO: LED on PA5 === */
    GPIO_Handle_t led;
    led.pGPIOx              = GPIOA;
    led.config.pin_number   = 5;
    led.config.mode         = GPIO_MODE_OUTPUT;
    led.config.speed        = GPIO_SPEED_FAST;
    led.config.output_type  = GPIO_OTYPE_PUSHPULL;
    led.config.pull         = GPIO_PUPD_NONE;
    GPIO_Init(&led);

    /* === UART: 115200 8N1 on USART2 === */
    UART_Handle_t uart;
    uart.pUSARTx             = USART2;
    uart.config.baud_rate    = 115200;
    uart.config.word_length  = UART_WORDLEN_8;
    uart.config.stop_bits    = UART_STOP_1;
    uart.config.parity       = UART_PARITY_NONE;
    uart.config.mode         = UART_MODE_TXRX;
    uart.config.hw_flow_control = UART_FLOW_NONE;
    UART_Init(&uart);
    UART_PeripheralControl(uart.pUSARTx, ENABLE);

    uint8_t msg[] = "Hello from bare-metal!\r\n";
    UART_Transmit(&uart, msg, sizeof(msg) - 1);

    while (1) {
        GPIO_TogglePin(GPIOA, 5);
        for (volatile int i = 0; i < 500000; i++);
    }
}
```

---

## 🔧 Target Hardware

| Parameter | Value |
|-----------|-------|
| MCU Family | STM32F4xx (F401, F411, F446) |
| Core | ARM Cortex-M4F |
| Default Clock | 16 MHz (HSI) |
| Priority Bits | 4 (16 levels) |
| Toolchain | ARM GCC / Keil MDK / IAR |

---

*This driver suite demonstrates the kind of low-level firmware that embedded engineers write at companies like Texas Instruments, STMicroelectronics, and Bosch Sensortec.*

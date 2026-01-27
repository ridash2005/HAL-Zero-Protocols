# STM32 Development Guide

If you are looking at the C files in the `examples/stm32/` directories and asking **"Where is main.h?"**, this guide is for you.

## 🟢 The Workflow

STM32 development is slightly different from Arduino. We rely on a tool called **STM32CubeMX** to generate the initialization code for the specific chip you are using (e.g., STM32F103, STM32F401, STM32H7).

### 1. Why is `main.h` missing?
`main.h` defines the specific hardware mapping for **your** board (e.g., Which pin is the LED? Which pin is the button?). Since we don't know which specific STM32 board you have, we cannot provide this file. **It is generated automatically by CubeMX.**

### 2. How to use the Example Code

1.  **Open STM32CubeMX**: Create a new project for your MCU.
2.  **Configure Peripherals**:
    *   **UART**: Enable `USART2` (or similar) -> Asynchronous.
    *   **SPI**: Enable `SPI1` -> Full-Duplex Master.
    *   **I2C**: Enable `I2C1` -> I2C.
3.  **Generate Code**: Click "Generate Code". This allows CubeMX to create the file structure:
    ```text
    MyProject/
    ├── Core/
    │   ├── Inc/
    │   │   ├── main.h        <-- IT IS HERE!
    │   │   └── stm32f4xx_hal_conf.h
    │   └── Src/
    │       ├── main.c        <-- The file you edit
    │       └── stm32f4xx_hal_msp.c
    ```
4.  **Copy the Logic**:
    *   Open the `examples/stm32/main.c` file from this repository.
    *   Copy the code from the `main()` function (and any helper functions).
    *   Paste it into the **User Code Begin** sections of *your* generated `main.c`.

### 3. Understanding `main.h`
When generated, `main.h` contains macros like:
```c
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
```
Our examples rely on you having a valid project environment where these standard HAL headers are available.

/*
 * STM32 HAL I2C Scanner Example
 *
 * This program scans the I2C bus for active devices.
 *
 * Hardware Setup:
 * - STM32 Board (Master)
 * - I2C Device (Slave) connected to SCL/SDA
 * - Pull-up resistors (4.7k) on SCL and SDA lines (Crucial!)
 *
 * CubeMX Configuration:
 * 1. Connectivity -> I2C1 -> I2C.
 * 2. Parameters -> Standard Mode (100kHz) or Fast Mode (400kHz).
 * 3. Generate Code.
 */

#include "main.h"
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2; // Used for printing results to PC

/* Function Prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);

void UART_Printf(const char *fmt, ...);

int main(void) {
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();

  char msg[64];
  UART_Printf("\r\n--- STM32 I2C Scanner ---\r\n");

  while (1) {
    UART_Printf("Scanning I2C bus...\r\n");
    int devicesFound = 0;

    for (uint16_t i = 1; i < 128; i++) {
      /*
       * HAL_I2C_IsDeviceReady checks if a device acknowledges its address.
       * Parameters: Handle, DevAddress (shifted left by 1), Trials, Timeout
       */
      HAL_StatusTypeDef result =
          HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i << 1), 2, 2);

      if (result == HAL_OK) {
        sprintf(msg, "Device found at 0x%02X\r\n", i);
        HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
        devicesFound++;
      }
    }

    if (devicesFound == 0) {
      UART_Printf("No devices found.\r\n");
    } else {
      UART_Printf("Scan complete.\r\n");
    }

    HAL_Delay(5000); // 5 Seconds wait
  }
}

/* Helper to print formatted strings to UART */
#include <stdarg.h>
void UART_Printf(const char *fmt, ...) {
  char buff[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buff, sizeof(buff), fmt, args);
  HAL_UART_Transmit(&huart2, (uint8_t *)buff, strlen(buff), 100);
  va_end(args);
}

static void MX_I2C1_Init(void) {
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    // Error_Handler();
  }
}

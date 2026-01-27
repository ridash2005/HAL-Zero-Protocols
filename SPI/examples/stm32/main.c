/*
 * STM32 HAL SPI Master Example
 *
 * This file demonstrates sending data via SPI as a Master.
 *
 * Hardware Setup:
 * - STM32 Board acting as Master.
 * - Logic Analyzer or another SPI Slave device to verify.
 *
 * CubeMX Configuration:
 * 1. Connectivity -> SPI1 -> Mode: Full-Duplex Master.
 * 2. Hardware NSS Signal: Disable (We will control Chip Select via GPIO
 * manually for flexibility).
 * 3. Parameters:
 *    - Data Size: 8 Bits
 *    - First Bit: MSB First
 *    - Prescaler: Set to get reasonable baud rate (e.g., 1MBits/s)
 *    - CPOL: Low (0)
 *    - CPHA: 1 Edge (0) -> Mode 0
 * 4. System Core -> GPIO -> Add a pin (e.g., PA4) as GPIO_Output for Chip
 * Select (CS).
 */

#include "main.h"
#include <string.h>

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* Macros */
#define CS_PORT GPIOA
#define CS_PIN GPIO_PIN_4

/* Function Prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

int main(void) {
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_SPI1_Init();

  /* Set CS High (Inactive) initially */
  HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);

  uint8_t dataToSend[] = {0xAA, 0xBB, 0xCC, 0x12, 0x34};

  while (1) {
    /* 1. Pull CS Low to select Slave */
    HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);

    /* 2. Transmit Data (Blocking mode) */
    /* Note: For Receive, use HAL_SPI_Receive or HAL_SPI_TransmitReceive */
    HAL_SPI_Transmit(&hspi1, dataToSend, 5, 100);

    /* 3. Pull CS High to deselect Slave */
    HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);

    /* Wait before next transmission */
    HAL_Delay(500);
  }
}

static void MX_SPI1_Init(void) {
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    // Error_Handler();
  }
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 (CS) */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

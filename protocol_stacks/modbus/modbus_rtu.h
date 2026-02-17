/**
 * @file    modbus_rtu.h
 * @brief   MODBUS RTU Protocol Stack — Public API
 * @version 1.0.0
 *
 * @details Complete MODBUS RTU slave implementation following the
 *          Modbus Application Protocol Specification V1.1b3.
 *
 * Features:
 *          - Full frame parsing with CRC-16 validation
 *          - Slave responder (responds to master queries)
 *          - Supported function codes:
 *            • 0x01 — Read Coils
 *            • 0x02 — Read Discrete Inputs
 *            • 0x03 — Read Holding Registers
 *            • 0x04 — Read Input Registers
 *            • 0x05 — Write Single Coil
 *            • 0x06 — Write Single Register
 *            • 0x0F — Write Multiple Coils
 *            • 0x10 — Write Multiple Registers
 *          - Exception response generation
 *          - Configurable slave address and data tables
 *          - Inter-frame timeout detection (3.5 character times)
 *
 * Usage in Industrial Automation:
 *          MODBUS RTU is the de-facto standard for PLC communication.
 *          This implementation can run on any STM32 with a UART peripheral
 *          connected to an RS-485 transceiver.
 */

#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include <stddef.h>
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 *  Configuration — adjust these for your application
 * ═══════════════════════════════════════════════════════════════════════════
 */

#ifndef MODBUS_MAX_PDU_SIZE
#define MODBUS_MAX_PDU_SIZE 253 /*!< Max PDU size (MODBUS spec)    */
#endif

#ifndef MODBUS_MAX_FRAME_SIZE
#define MODBUS_MAX_FRAME_SIZE 256 /*!< Address + PDU + CRC          */
#endif

/* Data table sizes (coils, registers, etc.) */
#ifndef MODBUS_NUM_COILS
#define MODBUS_NUM_COILS 256 /*!< Digital outputs (R/W)        */
#endif

#ifndef MODBUS_NUM_DISCRETE_INPUTS
#define MODBUS_NUM_DISCRETE_INPUTS 256 /*!< Digital inputs (Read-only)   */
#endif

#ifndef MODBUS_NUM_HOLDING_REGS
#define MODBUS_NUM_HOLDING_REGS 128 /*!< Analog outputs (R/W)         */
#endif

#ifndef MODBUS_NUM_INPUT_REGS
#define MODBUS_NUM_INPUT_REGS 128 /*!< Analog inputs (Read-only)    */
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 *  MODBUS Function Codes
 * ═══════════════════════════════════════════════════════════════════════════
 */

typedef enum {
  MODBUS_FC_READ_COILS = 0x01,
  MODBUS_FC_READ_DISCRETE_INPUTS = 0x02,
  MODBUS_FC_READ_HOLDING_REGISTERS = 0x03,
  MODBUS_FC_READ_INPUT_REGISTERS = 0x04,
  MODBUS_FC_WRITE_SINGLE_COIL = 0x05,
  MODBUS_FC_WRITE_SINGLE_REGISTER = 0x06,
  MODBUS_FC_WRITE_MULTIPLE_COILS = 0x0F,
  MODBUS_FC_WRITE_MULTIPLE_REGISTERS = 0x10,
} modbus_function_code_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  MODBUS Exception Codes
 * ═══════════════════════════════════════════════════════════════════════════
 */

typedef enum {
  MODBUS_EX_NONE = 0x00,
  MODBUS_EX_ILLEGAL_FUNCTION = 0x01,
  MODBUS_EX_ILLEGAL_DATA_ADDRESS = 0x02,
  MODBUS_EX_ILLEGAL_DATA_VALUE = 0x03,
  MODBUS_EX_SLAVE_DEVICE_FAILURE = 0x04,
  MODBUS_EX_ACKNOWLEDGE = 0x05,
  MODBUS_EX_SLAVE_DEVICE_BUSY = 0x06,
} modbus_exception_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Error / Status Codes
 * ═══════════════════════════════════════════════════════════════════════════
 */

typedef enum {
  MODBUS_OK = 0,
  MODBUS_ERR_CRC = -1,     /*!< CRC check failed              */
  MODBUS_ERR_FRAME = -2,   /*!< Invalid frame length           */
  MODBUS_ERR_ADDR = -3,    /*!< Address mismatch               */
  MODBUS_ERR_FUNC = -4,    /*!< Unsupported function code      */
  MODBUS_ERR_PARAM = -5,   /*!< Invalid parameter              */
  MODBUS_ERR_TIMEOUT = -6, /*!< Inter-frame timeout            */
} modbus_status_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Data Tables
 * ═══════════════════════════════════════════════════════════════════════════
 */

/**
 * @brief  MODBUS data model (The "memory map" that the master accesses).
 *
 * In MODBUS, data is organized into four tables:
 *   - Coils:            1-bit R/W  (digital outputs)
 *   - Discrete Inputs:  1-bit RO   (digital inputs)
 *   - Holding Registers: 16-bit R/W (analog outputs / parameters)
 *   - Input Registers:  16-bit RO  (analog inputs / measurements)
 */
typedef struct {
  uint8_t coils[MODBUS_NUM_COILS / 8]; /*!< Bit-packed coils         */
  uint8_t
      discrete_inputs[MODBUS_NUM_DISCRETE_INPUTS / 8]; /*!< Bit-packed inputs */
  uint16_t holding_registers[MODBUS_NUM_HOLDING_REGS]; /*!< Holding registers */
  uint16_t input_registers[MODBUS_NUM_INPUT_REGS];     /*!< Input registers     */
} modbus_data_table_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Slave Context
 * ═══════════════════════════════════════════════════════════════════════════
 */

/**
 * @brief  Hardware abstraction — the application must implement these.
 */
typedef struct {
  void (*uart_transmit)(const uint8_t *data,
                        uint16_t len); /*!< Send response frame  */
  void (*rs485_dir_tx)(void); /*!< Set RS-485 transceiver to TX mode  */
  void (*rs485_dir_rx)(void); /*!< Set RS-485 transceiver to RX mode  */
} modbus_hal_t;

/**
 * @brief  MODBUS RTU slave context.
 */
typedef struct {
  uint8_t slave_address;     /*!< This slave's address (1-247)  */
  modbus_data_table_t *data; /*!< Pointer to data tables        */
  modbus_hal_t hal;          /*!< Hardware abstraction layer     */

  /* Internal state for frame assembly */
  uint8_t rx_buffer[MODBUS_MAX_FRAME_SIZE];
  uint16_t rx_count;
  uint8_t tx_buffer[MODBUS_MAX_FRAME_SIZE];
} modbus_slave_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════════════
 */

/**
 * @brief  Initialize the MODBUS RTU slave.
 * @param  ctx: Pointer to slave context
 * @param  address: Slave address (1-247)
 * @param  data: Pointer to data table storage
 * @param  hal: Hardware abstraction callbacks
 * @retval modbus_status_t
 */
modbus_status_t MODBUS_Init(modbus_slave_t *ctx, uint8_t address,
                            modbus_data_table_t *data, modbus_hal_t hal);

/**
 * @brief  Feed received UART bytes into the MODBUS state machine.
 *         Call this from your UART RX ISR or polling loop.
 * @param  ctx: Slave context
 * @param  byte: Received byte
 */
void MODBUS_ReceiveByte(modbus_slave_t *ctx, uint8_t byte);

/**
 * @brief  Process a complete MODBUS frame.
 *         Call this after detecting the inter-frame silence (3.5 char times).
 * @param  ctx: Slave context
 * @retval modbus_status_t
 *
 * @note   This function validates CRC, checks the address, dispatches the
 *         function code handler, and transmits the response.
 */
modbus_status_t MODBUS_ProcessFrame(modbus_slave_t *ctx);

/**
 * @brief  Reset the receive buffer (e.g., after a timeout).
 * @param  ctx: Slave context
 */
void MODBUS_ResetReceiver(modbus_slave_t *ctx);

/* ═══════════════════════════════════════════════════════════════════════════
 *  CRC-16 Utility (MODBUS variant)
 * ═══════════════════════════════════════════════════════════════════════════
 */

/**
 * @brief  Calculate the CRC-16 for a MODBUS RTU frame.
 * @param  data: Pointer to data buffer
 * @param  length: Number of bytes
 * @retval CRC-16 value (little-endian, as per MODBUS convention)
 */
uint16_t MODBUS_CRC16(const uint8_t *data, uint16_t length);

/* ═══════════════════════════════════════════════════════════════════════════
 *  Data Table Access Helpers
 * ═══════════════════════════════════════════════════════════════════════════
 */

void MODBUS_SetCoil(modbus_data_table_t *data, uint16_t address, uint8_t value);
uint8_t MODBUS_GetCoil(modbus_data_table_t *data, uint16_t address);
void MODBUS_SetDiscreteInput(modbus_data_table_t *data, uint16_t address,
                             uint8_t value);
uint8_t MODBUS_GetDiscreteInput(modbus_data_table_t *data, uint16_t address);
void MODBUS_SetHoldingRegister(modbus_data_table_t *data, uint16_t address,
                               uint16_t value);
uint16_t MODBUS_GetHoldingRegister(modbus_data_table_t *data, uint16_t address);
void MODBUS_SetInputRegister(modbus_data_table_t *data, uint16_t address,
                             uint16_t value);
uint16_t MODBUS_GetInputRegister(modbus_data_table_t *data, uint16_t address);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_RTU_H */

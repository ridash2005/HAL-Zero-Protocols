/**
 * @file    modbus_rtu.c
 * @brief   MODBUS RTU Protocol Stack — Implementation
 * @version 1.0.0
 *
 * @details Full MODBUS RTU slave responder with CRC-16 validation,
 *          function code dispatching, and exception response generation.
 *
 * Frame Format (RTU):
 *   [Address(1)] [Function(1)] [Data(0-252)] [CRC_Lo(1)] [CRC_Hi(1)]
 *
 * Timing:
 *   - Inter-character gap: max 1.5 character times
 *   - Inter-frame silence: min 3.5 character times
 *   At 9600 baud: 1 char ≈ 1.04 ms → 3.5 char ≈ 3.65 ms
 */

#include "modbus_rtu.h"
#include <string.h>

/* ═══════════════════════════════════════════════════════════════════════════
 *  CRC-16 (MODBUS Variant — Polynomial 0xA001, reflected)
 * ═══════════════════════════════════════════════════════════════════════════
 */

/**
 * @brief  CRC-16/MODBUS calculation using the bit-shift algorithm.
 *
 * Polynomial: 0x8005 (reflected = 0xA001)
 * Initial value: 0xFFFF
 * Result is appended in little-endian order (CRC_Lo first, CRC_Hi second).
 */
uint16_t MODBUS_CRC16(const uint8_t *data, uint16_t length) {
  uint16_t crc = 0xFFFF;

  for (uint16_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i];

    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }

  return crc;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Data Table Access
 * ═══════════════════════════════════════════════════════════════════════════
 */

void MODBUS_SetCoil(modbus_data_table_t *data, uint16_t address,
                    uint8_t value) {
  if (address < MODBUS_NUM_COILS) {
    uint16_t byte_idx = address / 8;
    uint8_t bit_idx = address % 8;
    if (value) {
      data->coils[byte_idx] |= (1 << bit_idx);
    } else {
      data->coils[byte_idx] &= ~(1 << bit_idx);
    }
  }
}

uint8_t MODBUS_GetCoil(modbus_data_table_t *data, uint16_t address) {
  if (address < MODBUS_NUM_COILS) {
    uint16_t byte_idx = address / 8;
    uint8_t bit_idx = address % 8;
    return (data->coils[byte_idx] >> bit_idx) & 0x01;
  }
  return 0;
}

void MODBUS_SetDiscreteInput(modbus_data_table_t *data, uint16_t address,
                             uint8_t value) {
  if (address < MODBUS_NUM_DISCRETE_INPUTS) {
    uint16_t byte_idx = address / 8;
    uint8_t bit_idx = address % 8;
    if (value) {
      data->discrete_inputs[byte_idx] |= (1 << bit_idx);
    } else {
      data->discrete_inputs[byte_idx] &= ~(1 << bit_idx);
    }
  }
}

uint8_t MODBUS_GetDiscreteInput(modbus_data_table_t *data, uint16_t address) {
  if (address < MODBUS_NUM_DISCRETE_INPUTS) {
    uint16_t byte_idx = address / 8;
    uint8_t bit_idx = address % 8;
    return (data->discrete_inputs[byte_idx] >> bit_idx) & 0x01;
  }
  return 0;
}

void MODBUS_SetHoldingRegister(modbus_data_table_t *data, uint16_t address,
                               uint16_t value) {
  if (address < MODBUS_NUM_HOLDING_REGS) {
    data->holding_registers[address] = value;
  }
}

uint16_t MODBUS_GetHoldingRegister(modbus_data_table_t *data,
                                   uint16_t address) {
  if (address < MODBUS_NUM_HOLDING_REGS) {
    return data->holding_registers[address];
  }
  return 0;
}

void MODBUS_SetInputRegister(modbus_data_table_t *data, uint16_t address,
                             uint16_t value) {
  if (address < MODBUS_NUM_INPUT_REGS) {
    data->input_registers[address] = value;
  }
}

uint16_t MODBUS_GetInputRegister(modbus_data_table_t *data, uint16_t address) {
  if (address < MODBUS_NUM_INPUT_REGS) {
    return data->input_registers[address];
  }
  return 0;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Internal — Build Exception Response
 * ═══════════════════════════════════════════════════════════════════════════
 */

static uint16_t MODBUS_BuildException(modbus_slave_t *ctx,
                                      uint8_t function_code,
                                      modbus_exception_t exception) {
  ctx->tx_buffer[0] = ctx->slave_address;
  ctx->tx_buffer[1] = function_code | 0x80; /* Error flag (bit 7 set) */
  ctx->tx_buffer[2] = (uint8_t)exception;

  uint16_t crc = MODBUS_CRC16(ctx->tx_buffer, 3);
  ctx->tx_buffer[3] = (uint8_t)(crc & 0xFF);        /* CRC Lo */
  ctx->tx_buffer[4] = (uint8_t)((crc >> 8) & 0xFF); /* CRC Hi */

  return 5;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Internal — Function Code Handlers
 * ═══════════════════════════════════════════════════════════════════════════
 */

/**
 * @brief  FC 0x03: Read Holding Registers / FC 0x04: Read Input Registers
 */
static uint16_t MODBUS_HandleReadRegisters(modbus_slave_t *ctx, uint8_t fc,
                                           uint8_t *pdu, uint16_t pdu_len) {
  if (pdu_len < 4) {
    return MODBUS_BuildException(ctx, fc, MODBUS_EX_ILLEGAL_DATA_VALUE);
  }

  uint16_t start_addr = ((uint16_t)pdu[0] << 8) | pdu[1];
  uint16_t quantity = ((uint16_t)pdu[2] << 8) | pdu[3];

  /* Validate quantity (1-125 registers) */
  if (quantity < 1 || quantity > 125) {
    return MODBUS_BuildException(ctx, fc, MODBUS_EX_ILLEGAL_DATA_VALUE);
  }

  /* Validate address range */
  uint16_t max_regs = (fc == MODBUS_FC_READ_HOLDING_REGISTERS)
                          ? MODBUS_NUM_HOLDING_REGS
                          : MODBUS_NUM_INPUT_REGS;
  if ((start_addr + quantity) > max_regs) {
    return MODBUS_BuildException(ctx, fc, MODBUS_EX_ILLEGAL_DATA_ADDRESS);
  }

  /* Build response */
  uint8_t byte_count = (uint8_t)(quantity * 2);
  ctx->tx_buffer[0] = ctx->slave_address;
  ctx->tx_buffer[1] = fc;
  ctx->tx_buffer[2] = byte_count;

  for (uint16_t i = 0; i < quantity; i++) {
    uint16_t reg_val;
    if (fc == MODBUS_FC_READ_HOLDING_REGISTERS) {
      reg_val = ctx->data->holding_registers[start_addr + i];
    } else {
      reg_val = ctx->data->input_registers[start_addr + i];
    }
    ctx->tx_buffer[3 + (i * 2)] = (uint8_t)(reg_val >> 8);       /* Hi byte */
    ctx->tx_buffer[3 + (i * 2) + 1] = (uint8_t)(reg_val & 0xFF); /* Lo byte */
  }

  uint16_t frame_len = 3 + byte_count;
  uint16_t crc = MODBUS_CRC16(ctx->tx_buffer, frame_len);
  ctx->tx_buffer[frame_len] = (uint8_t)(crc & 0xFF);
  ctx->tx_buffer[frame_len + 1] = (uint8_t)((crc >> 8) & 0xFF);

  return frame_len + 2;
}

/**
 * @brief  FC 0x01: Read Coils / FC 0x02: Read Discrete Inputs
 */
static uint16_t MODBUS_HandleReadBits(modbus_slave_t *ctx, uint8_t fc,
                                      uint8_t *pdu, uint16_t pdu_len) {
  if (pdu_len < 4) {
    return MODBUS_BuildException(ctx, fc, MODBUS_EX_ILLEGAL_DATA_VALUE);
  }

  uint16_t start_addr = ((uint16_t)pdu[0] << 8) | pdu[1];
  uint16_t quantity = ((uint16_t)pdu[2] << 8) | pdu[3];

  if (quantity < 1 || quantity > 2000) {
    return MODBUS_BuildException(ctx, fc, MODBUS_EX_ILLEGAL_DATA_VALUE);
  }

  uint16_t max_bits = (fc == MODBUS_FC_READ_COILS) ? MODBUS_NUM_COILS
                                                   : MODBUS_NUM_DISCRETE_INPUTS;
  if ((start_addr + quantity) > max_bits) {
    return MODBUS_BuildException(ctx, fc, MODBUS_EX_ILLEGAL_DATA_ADDRESS);
  }

  uint8_t byte_count = (uint8_t)((quantity + 7) / 8);
  ctx->tx_buffer[0] = ctx->slave_address;
  ctx->tx_buffer[1] = fc;
  ctx->tx_buffer[2] = byte_count;

  /* Pack bits into response bytes */
  memset(&ctx->tx_buffer[3], 0, byte_count);
  for (uint16_t i = 0; i < quantity; i++) {
    uint8_t bit_val;
    if (fc == MODBUS_FC_READ_COILS) {
      bit_val = MODBUS_GetCoil(ctx->data, start_addr + i);
    } else {
      bit_val = MODBUS_GetDiscreteInput(ctx->data, start_addr + i);
    }
    if (bit_val) {
      ctx->tx_buffer[3 + (i / 8)] |= (1 << (i % 8));
    }
  }

  uint16_t frame_len = 3 + byte_count;
  uint16_t crc = MODBUS_CRC16(ctx->tx_buffer, frame_len);
  ctx->tx_buffer[frame_len] = (uint8_t)(crc & 0xFF);
  ctx->tx_buffer[frame_len + 1] = (uint8_t)((crc >> 8) & 0xFF);

  return frame_len + 2;
}

/**
 * @brief  FC 0x06: Write Single Register
 */
static uint16_t MODBUS_HandleWriteSingleRegister(modbus_slave_t *ctx,
                                                 uint8_t *pdu,
                                                 uint16_t pdu_len) {
  if (pdu_len < 4) {
    return MODBUS_BuildException(ctx, MODBUS_FC_WRITE_SINGLE_REGISTER,
                                 MODBUS_EX_ILLEGAL_DATA_VALUE);
  }

  uint16_t reg_addr = ((uint16_t)pdu[0] << 8) | pdu[1];
  uint16_t reg_val = ((uint16_t)pdu[2] << 8) | pdu[3];

  if (reg_addr >= MODBUS_NUM_HOLDING_REGS) {
    return MODBUS_BuildException(ctx, MODBUS_FC_WRITE_SINGLE_REGISTER,
                                 MODBUS_EX_ILLEGAL_DATA_ADDRESS);
  }

  ctx->data->holding_registers[reg_addr] = reg_val;

  /* Response is an echo of the request */
  ctx->tx_buffer[0] = ctx->slave_address;
  ctx->tx_buffer[1] = MODBUS_FC_WRITE_SINGLE_REGISTER;
  ctx->tx_buffer[2] = pdu[0];
  ctx->tx_buffer[3] = pdu[1];
  ctx->tx_buffer[4] = pdu[2];
  ctx->tx_buffer[5] = pdu[3];

  uint16_t crc = MODBUS_CRC16(ctx->tx_buffer, 6);
  ctx->tx_buffer[6] = (uint8_t)(crc & 0xFF);
  ctx->tx_buffer[7] = (uint8_t)((crc >> 8) & 0xFF);

  return 8;
}

/**
 * @brief  FC 0x05: Write Single Coil
 */
static uint16_t MODBUS_HandleWriteSingleCoil(modbus_slave_t *ctx, uint8_t *pdu,
                                             uint16_t pdu_len) {
  if (pdu_len < 4) {
    return MODBUS_BuildException(ctx, MODBUS_FC_WRITE_SINGLE_COIL,
                                 MODBUS_EX_ILLEGAL_DATA_VALUE);
  }

  uint16_t coil_addr = ((uint16_t)pdu[0] << 8) | pdu[1];
  uint16_t coil_val = ((uint16_t)pdu[2] << 8) | pdu[3];

  if (coil_addr >= MODBUS_NUM_COILS) {
    return MODBUS_BuildException(ctx, MODBUS_FC_WRITE_SINGLE_COIL,
                                 MODBUS_EX_ILLEGAL_DATA_ADDRESS);
  }

  /* MODBUS spec: 0xFF00 = ON, 0x0000 = OFF */
  if (coil_val != 0xFF00 && coil_val != 0x0000) {
    return MODBUS_BuildException(ctx, MODBUS_FC_WRITE_SINGLE_COIL,
                                 MODBUS_EX_ILLEGAL_DATA_VALUE);
  }

  MODBUS_SetCoil(ctx->data, coil_addr, (coil_val == 0xFF00) ? 1 : 0);

  /* Echo request back */
  ctx->tx_buffer[0] = ctx->slave_address;
  ctx->tx_buffer[1] = MODBUS_FC_WRITE_SINGLE_COIL;
  memcpy(&ctx->tx_buffer[2], pdu, 4);

  uint16_t crc = MODBUS_CRC16(ctx->tx_buffer, 6);
  ctx->tx_buffer[6] = (uint8_t)(crc & 0xFF);
  ctx->tx_buffer[7] = (uint8_t)((crc >> 8) & 0xFF);

  return 8;
}

/**
 * @brief  FC 0x10: Write Multiple Registers
 */
static uint16_t MODBUS_HandleWriteMultipleRegisters(modbus_slave_t *ctx,
                                                    uint8_t *pdu,
                                                    uint16_t pdu_len) {
  if (pdu_len < 5) {
    return MODBUS_BuildException(ctx, MODBUS_FC_WRITE_MULTIPLE_REGISTERS,
                                 MODBUS_EX_ILLEGAL_DATA_VALUE);
  }

  uint16_t start_addr = ((uint16_t)pdu[0] << 8) | pdu[1];
  uint16_t quantity = ((uint16_t)pdu[2] << 8) | pdu[3];
  uint8_t byte_count = pdu[4];

  if (quantity < 1 || quantity > 123 || byte_count != (quantity * 2)) {
    return MODBUS_BuildException(ctx, MODBUS_FC_WRITE_MULTIPLE_REGISTERS,
                                 MODBUS_EX_ILLEGAL_DATA_VALUE);
  }

  if ((start_addr + quantity) > MODBUS_NUM_HOLDING_REGS) {
    return MODBUS_BuildException(ctx, MODBUS_FC_WRITE_MULTIPLE_REGISTERS,
                                 MODBUS_EX_ILLEGAL_DATA_ADDRESS);
  }

  /* Write register values */
  for (uint16_t i = 0; i < quantity; i++) {
    uint16_t val = ((uint16_t)pdu[5 + (i * 2)] << 8) | pdu[5 + (i * 2) + 1];
    ctx->data->holding_registers[start_addr + i] = val;
  }

  /* Build response: Address + FC + Start_Addr + Quantity */
  ctx->tx_buffer[0] = ctx->slave_address;
  ctx->tx_buffer[1] = MODBUS_FC_WRITE_MULTIPLE_REGISTERS;
  ctx->tx_buffer[2] = pdu[0]; /* Start addr Hi */
  ctx->tx_buffer[3] = pdu[1]; /* Start addr Lo */
  ctx->tx_buffer[4] = pdu[2]; /* Quantity Hi */
  ctx->tx_buffer[5] = pdu[3]; /* Quantity Lo */

  uint16_t crc = MODBUS_CRC16(ctx->tx_buffer, 6);
  ctx->tx_buffer[6] = (uint8_t)(crc & 0xFF);
  ctx->tx_buffer[7] = (uint8_t)((crc >> 8) & 0xFF);

  return 8;
}

/**
 * @brief  FC 0x0F: Write Multiple Coils
 */
static uint16_t MODBUS_HandleWriteMultipleCoils(modbus_slave_t *ctx,
                                                uint8_t *pdu,
                                                uint16_t pdu_len) {
  if (pdu_len < 5) {
    return MODBUS_BuildException(ctx, MODBUS_FC_WRITE_MULTIPLE_COILS,
                                 MODBUS_EX_ILLEGAL_DATA_VALUE);
  }

  uint16_t start_addr = ((uint16_t)pdu[0] << 8) | pdu[1];
  uint16_t quantity = ((uint16_t)pdu[2] << 8) | pdu[3];
  uint8_t byte_count = pdu[4];

  if (quantity < 1 || quantity > 1968 || byte_count != ((quantity + 7) / 8)) {
    return MODBUS_BuildException(ctx, MODBUS_FC_WRITE_MULTIPLE_COILS,
                                 MODBUS_EX_ILLEGAL_DATA_VALUE);
  }

  if ((start_addr + quantity) > MODBUS_NUM_COILS) {
    return MODBUS_BuildException(ctx, MODBUS_FC_WRITE_MULTIPLE_COILS,
                                 MODBUS_EX_ILLEGAL_DATA_ADDRESS);
  }

  /* Write coil values */
  for (uint16_t i = 0; i < quantity; i++) {
    uint8_t bit_val = (pdu[5 + (i / 8)] >> (i % 8)) & 0x01;
    MODBUS_SetCoil(ctx->data, start_addr + i, bit_val);
  }

  /* Build response */
  ctx->tx_buffer[0] = ctx->slave_address;
  ctx->tx_buffer[1] = MODBUS_FC_WRITE_MULTIPLE_COILS;
  memcpy(&ctx->tx_buffer[2], pdu, 4); /* Start + Quantity */

  uint16_t crc = MODBUS_CRC16(ctx->tx_buffer, 6);
  ctx->tx_buffer[6] = (uint8_t)(crc & 0xFF);
  ctx->tx_buffer[7] = (uint8_t)((crc >> 8) & 0xFF);

  return 8;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════════════
 */

modbus_status_t MODBUS_Init(modbus_slave_t *ctx, uint8_t address,
                            modbus_data_table_t *data, modbus_hal_t hal) {
  if (ctx == NULL || data == NULL || address < 1 || address > 247) {
    return MODBUS_ERR_PARAM;
  }

  ctx->slave_address = address;
  ctx->data = data;
  ctx->hal = hal;
  ctx->rx_count = 0;

  /* Initialize data tables to zero */
  memset(data, 0, sizeof(modbus_data_table_t));

  return MODBUS_OK;
}

void MODBUS_ReceiveByte(modbus_slave_t *ctx, uint8_t byte) {
  if (ctx->rx_count < MODBUS_MAX_FRAME_SIZE) {
    ctx->rx_buffer[ctx->rx_count++] = byte;
  }
  /* If buffer overflows, the frame will fail CRC check */
}

void MODBUS_ResetReceiver(modbus_slave_t *ctx) { ctx->rx_count = 0; }

modbus_status_t MODBUS_ProcessFrame(modbus_slave_t *ctx) {
  /* Minimum frame is 4 bytes: Address(1) + FC(1) + CRC(2) */
  if (ctx->rx_count < 4) {
    MODBUS_ResetReceiver(ctx);
    return MODBUS_ERR_FRAME;
  }

  /* ── 1. CRC Validation ────────────────────────────────────── */
  uint16_t received_crc = ((uint16_t)ctx->rx_buffer[ctx->rx_count - 1] << 8) |
                          ctx->rx_buffer[ctx->rx_count - 2];
  uint16_t calculated_crc = MODBUS_CRC16(ctx->rx_buffer, ctx->rx_count - 2);

  if (received_crc != calculated_crc) {
    MODBUS_ResetReceiver(ctx);
    return MODBUS_ERR_CRC;
  }

  /* ── 2. Address Check ─────────────────────────────────────── */
  uint8_t frame_addr = ctx->rx_buffer[0];

  if (frame_addr != ctx->slave_address && frame_addr != 0) {
    /* Not addressed to us (and not broadcast) */
    MODBUS_ResetReceiver(ctx);
    return MODBUS_ERR_ADDR;
  }

  /* ── 3. Extract Function Code and PDU ─────────────────────── */
  uint8_t fc = ctx->rx_buffer[1];
  uint8_t *pdu = &ctx->rx_buffer[2];
  uint16_t pdu_len = ctx->rx_count - 4; /* Subtract Addr(1) + FC(1) + CRC(2) */

  /* ── 4. Dispatch Function Code Handler ───────────────────── */
  uint16_t tx_len = 0;

  switch (fc) {
  case MODBUS_FC_READ_COILS:
  case MODBUS_FC_READ_DISCRETE_INPUTS:
    tx_len = MODBUS_HandleReadBits(ctx, fc, pdu, pdu_len);
    break;

  case MODBUS_FC_READ_HOLDING_REGISTERS:
  case MODBUS_FC_READ_INPUT_REGISTERS:
    tx_len = MODBUS_HandleReadRegisters(ctx, fc, pdu, pdu_len);
    break;

  case MODBUS_FC_WRITE_SINGLE_COIL:
    tx_len = MODBUS_HandleWriteSingleCoil(ctx, pdu, pdu_len);
    break;

  case MODBUS_FC_WRITE_SINGLE_REGISTER:
    tx_len = MODBUS_HandleWriteSingleRegister(ctx, pdu, pdu_len);
    break;

  case MODBUS_FC_WRITE_MULTIPLE_COILS:
    tx_len = MODBUS_HandleWriteMultipleCoils(ctx, pdu, pdu_len);
    break;

  case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
    tx_len = MODBUS_HandleWriteMultipleRegisters(ctx, pdu, pdu_len);
    break;

  default:
    tx_len = MODBUS_BuildException(ctx, fc, MODBUS_EX_ILLEGAL_FUNCTION);
    break;
  }

  /* ── 5. Transmit Response ─────────────────────────────────── */
  if (tx_len > 0 && frame_addr != 0) {
    /* Broadcast frames (addr 0) do not get a response */
    if (ctx->hal.rs485_dir_tx) {
      ctx->hal.rs485_dir_tx();
    }

    if (ctx->hal.uart_transmit) {
      ctx->hal.uart_transmit(ctx->tx_buffer, tx_len);
    }

    if (ctx->hal.rs485_dir_rx) {
      ctx->hal.rs485_dir_rx();
    }
  }

  MODBUS_ResetReceiver(ctx);
  return MODBUS_OK;
}

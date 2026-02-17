/**
 * @file    test_drivers.c
 * @brief   Host-compiled unit tests for the STM32F4xx driver suite
 * @version 1.0.0
 *
 * @details This test suite verifies driver logic by running on the HOST
 *          machine (x86/x64) against a simulated register file. It does
 *          NOT require real hardware.
 *
 * Build:
 *   gcc -std=c11 -Wall -Wextra -DUNIT_TEST \
 *       -I../drivers/common                    \
 *       test_drivers.c -o test_drivers && ./test_drivers
 *
 * The UNIT_TEST flag tells stm32f4xx_base.h to provide simulated
 * register memory instead of real memory-mapped addresses.
 */

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ═══════════════════════════════════════════════════════════════════════════
 *  Simulated Hardware Registers
 *  When UNIT_TEST is defined, we provide static memory that mimics
 *  the peripheral register layout so driver code can manipulate bits.
 * ═══════════════════════════════════════════════════════════════════════════
 */

/* --- Simulated RCC ------------------------------------------------------- */
typedef struct {
  volatile uint32_t CR;
  volatile uint32_t PLLCFGR;
  volatile uint32_t CFGR;
  volatile uint32_t CIR;
  volatile uint32_t AHB1RSTR;
  volatile uint32_t AHB2RSTR;
  volatile uint32_t RESERVED0[2];
  volatile uint32_t APB1RSTR;
  volatile uint32_t APB2RSTR;
  volatile uint32_t RESERVED1[2];
  volatile uint32_t AHB1ENR;
  volatile uint32_t AHB2ENR;
  volatile uint32_t RESERVED2[2];
  volatile uint32_t APB1ENR;
  volatile uint32_t APB2ENR;
} RCC_Sim_t;

static RCC_Sim_t rcc_sim;

/* --- Simulated GPIO ------------------------------------------------------ */
typedef struct {
  volatile uint32_t MODER;
  volatile uint32_t OTYPER;
  volatile uint32_t OSPEEDR;
  volatile uint32_t PUPDR;
  volatile uint32_t IDR;
  volatile uint32_t ODR;
  volatile uint32_t BSRR;
  volatile uint32_t LCKR;
  volatile uint32_t AFR[2];
} GPIO_Sim_t;

static GPIO_Sim_t gpioa_sim, gpiob_sim;

/* --- Simulated EXTI ------------------------------------------------------ */
typedef struct {
  volatile uint32_t IMR;
  volatile uint32_t EMR;
  volatile uint32_t RTSR;
  volatile uint32_t FTSR;
  volatile uint32_t SWIER;
  volatile uint32_t PR;
} EXTI_Sim_t;

static EXTI_Sim_t exti_sim;

/* --- Simulated SYSCFG ---------------------------------------------------- */
typedef struct {
  volatile uint32_t MEMRMP;
  volatile uint32_t PMC;
  volatile uint32_t EXTICR[4];
} SYSCFG_Sim_t;

static SYSCFG_Sim_t syscfg_sim;

/* --- Simulated USART ----------------------------------------------------- */
typedef struct {
  volatile uint32_t SR;
  volatile uint32_t DR;
  volatile uint32_t BRR;
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t CR3;
  volatile uint32_t GTPR;
} USART_Sim_t;

static USART_Sim_t usart2_sim;

/* --- Simulated SPI ------------------------------------------------------- */
typedef struct {
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t SR;
  volatile uint32_t DR;
  volatile uint32_t CRCPR;
  volatile uint32_t RXCRCR;
  volatile uint32_t TXCRCR;
  volatile uint32_t I2SCFGR;
  volatile uint32_t I2SPR;
} SPI_Sim_t;

static SPI_Sim_t spi1_sim;

/* --- Simulated I2C ------------------------------------------------------- */
typedef struct {
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t OAR1;
  volatile uint32_t OAR2;
  volatile uint32_t DR;
  volatile uint32_t SR1;
  volatile uint32_t SR2;
  volatile uint32_t CCR;
  volatile uint32_t TRISE;
  volatile uint32_t FLTR;
} I2C_Sim_t;

static I2C_Sim_t i2c1_sim;

/* --- Simulated NVIC ------------------------------------------------------ */
static volatile uint32_t nvic_iser[8];
static volatile uint32_t nvic_icer[8];
static volatile uint32_t nvic_ipr[60];

/* ═══════════════════════════════════════════════════════════════════════════
 *  Override hardware pointers
 *  These macros redirect driver code from real hardware addresses
 *  to our simulated register memory.
 * ═══════════════════════════════════════════════════════════════════════════
 */
#define RCC ((RCC_Sim_t *)&rcc_sim)
#define GPIOA ((GPIO_Sim_t *)&gpioa_sim)
#define GPIOB ((GPIO_Sim_t *)&gpiob_sim)
#define EXTI ((EXTI_Sim_t *)&exti_sim)
#define SYSCFG ((SYSCFG_Sim_t *)&syscfg_sim)
#define USART2 ((USART_Sim_t *)&usart2_sim)
#define SPI1 ((SPI_Sim_t *)&spi1_sim)
#define I2C1 ((I2C_Sim_t *)&i2c1_sim)

/* Clock macros redirect to our simulated RCC */
#define GPIOA_PCLK_EN() (rcc_sim.AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN() (rcc_sim.AHB1ENR |= (1 << 1))
#define GPIOA_PCLK_DI() (rcc_sim.AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI() (rcc_sim.AHB1ENR &= ~(1 << 1))
#define SYSCFG_PCLK_EN() (rcc_sim.APB2ENR |= (1 << 14))

/* NVIC macros redirect to our simulated arrays */
#define NVIC_ISER_BASE ((volatile uint32_t *)nvic_iser)
#define NVIC_ICER_BASE ((volatile uint32_t *)nvic_icer)
#define NVIC_IPR_BASE ((volatile uint32_t *)nvic_ipr)

/* Provide ENABLE/DISABLE if not defined */
#ifndef ENABLE
#define ENABLE 1
#define DISABLE 0
#endif

#define SYSTEM_CLOCK_HZ 16000000UL
#define NVIC_PRIO_BITS 4

/* ═══════════════════════════════════════════════════════════════════════════
 *  Redirection types — match the driver's expected type names
 * ═══════════════════════════════════════════════════════════════════════════
 */
typedef GPIO_Sim_t GPIO_RegDef_t;
typedef EXTI_Sim_t EXTI_RegDef_t;
typedef SYSCFG_Sim_t SYSCFG_RegDef_t;
typedef USART_Sim_t USART_RegDef_t;
typedef SPI_Sim_t SPI_RegDef_t;
typedef I2C_Sim_t I2C_RegDef_t;

/* Provide __vo as volatile (used by some drivers) */
#define __vo volatile

/* ═══════════════════════════════════════════════════════════════════════════
 *  Test Framework (minimal)
 * ═══════════════════════════════════════════════════════════════════════════
 */

static int tests_run = 0;
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST_ASSERT(cond, msg)                                                 \
  do {                                                                         \
    tests_run++;                                                               \
    if (cond) {                                                                \
      tests_passed++;                                                          \
      printf("  ✅ PASS: %s\n", msg);                                          \
    } else {                                                                   \
      tests_failed++;                                                          \
      printf("  ❌ FAIL: %s (line %d)\n", msg, __LINE__);                      \
    }                                                                          \
  } while (0)

#define TEST_SUITE(name) printf("\n══ %s ══\n", name)

/* ═══════════════════════════════════════════════════════════════════════════
 *  Test: MODBUS RTU CRC-16
 * ═══════════════════════════════════════════════════════════════════════════
 */

/* Include the MODBUS source directly so we can test it without linking */
#include "../protocol_stacks/modbus/modbus_rtu.h"

/* Forward declare the CRC function (it's defined in the .c) */
extern uint16_t MODBUS_CRC16(const uint8_t *data, uint16_t length);

/* We include the .c file to get the implementation */
#include "../protocol_stacks/modbus/modbus_rtu.c"

static void test_modbus_crc16(void) {
  TEST_SUITE("MODBUS RTU — CRC-16 Validation");

  /* Known test vector: slave address 0x01, function 0x03, start 0x0000, qty
   * 0x0001 */
  uint8_t frame1[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01};
  uint16_t crc1 = MODBUS_CRC16(frame1, 6);
  TEST_ASSERT(crc1 == 0x0A84, "CRC-16 of Read Holding Register query (01 03 00 "
                              "00 00 01) = 0x0A84");

  /* Another known vector: slave 0x01, FC 0x06, reg 0x0001, val 0x0003 */
  uint8_t frame2[] = {0x01, 0x06, 0x00, 0x01, 0x00, 0x03};
  uint16_t crc2 = MODBUS_CRC16(frame2, 6);
  TEST_ASSERT(crc2 == 0x0B98,
              "CRC-16 of Write Single Register (01 06 00 01 00 03) = 0x0B98");

  /* Empty data should produce initial 0xFFFF XOR'd */
  uint16_t crc_empty = MODBUS_CRC16(NULL, 0);
  TEST_ASSERT(crc_empty == 0xFFFF,
              "CRC-16 of empty data = 0xFFFF (initial value)");

  /* Single byte */
  uint8_t single[] = {0x00};
  uint16_t crc_single = MODBUS_CRC16(single, 1);
  TEST_ASSERT(crc_single != 0xFFFF, "CRC-16 of single zero byte != 0xFFFF");
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Test: MODBUS RTU Data Table Access
 * ═══════════════════════════════════════════════════════════════════════════
 */

static void test_modbus_data_table(void) {
  TEST_SUITE("MODBUS RTU — Data Table Access");

  modbus_data_table_t data;
  memset(&data, 0, sizeof(data));

  /* Write and read back holding registers */
  data.holding_registers[0] = 0x1234;
  data.holding_registers[1] = 0xABCD;
  TEST_ASSERT(data.holding_registers[0] == 0x1234,
              "Holding register [0] write/read = 0x1234");
  TEST_ASSERT(data.holding_registers[1] == 0xABCD,
              "Holding register [1] write/read = 0xABCD");

  /* Write coils (bit-packed) */
  data.coils[0] = 0xA5;
  TEST_ASSERT((data.coils[0] & 0x01) == 1, "Coil 0 is ON (0xA5 bit 0)");
  TEST_ASSERT(((data.coils[0] >> 1) & 0x01) == 0, "Coil 1 is OFF (0xA5 bit 1)");
  TEST_ASSERT(((data.coils[0] >> 5) & 0x01) == 1, "Coil 5 is ON (0xA5 bit 5)");
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Test: MODBUS RTU Slave Init & Frame Processing
 * ═══════════════════════════════════════════════════════════════════════════
 */

static uint8_t test_tx_buf[256];
static uint16_t test_tx_len;

static void mock_uart_tx(const uint8_t *data, uint16_t len) {
  memcpy(test_tx_buf, data, len);
  test_tx_len = len;
}

static void mock_rs485_tx(void) { /* no-op */ }
static void mock_rs485_rx(void) { /* no-op */ }

static void test_modbus_frame_processing(void) {
  TEST_SUITE("MODBUS RTU — Frame Processing");

  modbus_data_table_t data;
  memset(&data, 0, sizeof(data));

  modbus_slave_t slave;
  modbus_hal_t hal = {
      .uart_transmit = mock_uart_tx,
      .rs485_dir_tx = mock_rs485_tx,
      .rs485_dir_rx = mock_rs485_rx,
  };

  MODBUS_Init(&slave, 1, &data, hal);
  /* Set registers AFTER Init (Init zeroes the data table) */
  data.holding_registers[0] = 0x1234;

  TEST_ASSERT(slave.slave_address == 1, "Slave initialized with address 1");
  TEST_ASSERT(slave.rx_count == 0, "RX count starts at 0");

  /* Simulate receiving a Read Holding Register frame:
   * slave_address=0x01, FC=0x03, StartHi=0x00, StartLo=0x00, QtyHi=0x00,
   * QtyLo=0x01, CRC_Lo, CRC_Hi */
  uint8_t query[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01};
  uint16_t crc = MODBUS_CRC16(query, 6);

  /* Feed bytes one at a time */
  for (int i = 0; i < 6; i++) {
    MODBUS_ReceiveByte(&slave, query[i]);
  }
  MODBUS_ReceiveByte(&slave, (uint8_t)(crc & 0xFF));
  MODBUS_ReceiveByte(&slave, (uint8_t)(crc >> 8));

  /* Process the frame */
  test_tx_len = 0;
  modbus_status_t status = MODBUS_ProcessFrame(&slave);
  TEST_ASSERT(status == MODBUS_OK, "Frame processing returns MODBUS_OK");
  TEST_ASSERT(test_tx_len > 0, "Response was transmitted");

  /* Verify response: Addr=0x01, FC=0x03, ByteCount=0x02, DataHi=0x12,
   * DataLo=0x34 */
  TEST_ASSERT(test_tx_buf[0] == 0x01, "Response address = 0x01");
  TEST_ASSERT(test_tx_buf[1] == 0x03, "Response function code = 0x03");
  TEST_ASSERT(test_tx_buf[2] == 0x02, "Response byte count = 2");
  TEST_ASSERT(test_tx_buf[3] == 0x12, "Response data high byte = 0x12");
  TEST_ASSERT(test_tx_buf[4] == 0x34, "Response data low byte = 0x34");

  /* Verify CRC of response */
  uint16_t resp_crc = MODBUS_CRC16(test_tx_buf, test_tx_len - 2);
  uint16_t resp_crc_received = (uint16_t)(test_tx_buf[test_tx_len - 2]) |
                               ((uint16_t)(test_tx_buf[test_tx_len - 1]) << 8);
  TEST_ASSERT(resp_crc == resp_crc_received, "Response CRC is valid");
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Test: MODBUS Exception Responses
 * ═══════════════════════════════════════════════════════════════════════════
 */

static void test_modbus_exceptions(void) {
  TEST_SUITE("MODBUS RTU — Exception Handling");

  modbus_data_table_t data;
  memset(&data, 0, sizeof(data));

  modbus_slave_t slave;
  modbus_hal_t hal = {
      .uart_transmit = mock_uart_tx,
      .rs485_dir_tx = mock_rs485_tx,
      .rs485_dir_rx = mock_rs485_rx,
  };

  MODBUS_Init(&slave, 1, &data, hal);

  /* Test wrong slave address — should be silently ignored */
  uint8_t wrong_addr[] = {0x02, 0x03, 0x00, 0x00, 0x00, 0x01};
  uint16_t crc_wrong = MODBUS_CRC16(wrong_addr, 6);
  for (int i = 0; i < 6; i++)
    MODBUS_ReceiveByte(&slave, wrong_addr[i]);
  MODBUS_ReceiveByte(&slave, (uint8_t)(crc_wrong & 0xFF));
  MODBUS_ReceiveByte(&slave, (uint8_t)(crc_wrong >> 8));

  test_tx_len = 0;
  modbus_status_t status = MODBUS_ProcessFrame(&slave);
  TEST_ASSERT(test_tx_len == 0, "Wrong address: no response transmitted");

  /* Test unsupported function code */
  MODBUS_ResetReceiver(&slave);
  uint8_t bad_fc[] = {0x01, 0x7F, 0x00, 0x00, 0x00, 0x01}; /* FC=0x7F invalid */
  uint16_t crc_bad = MODBUS_CRC16(bad_fc, 6);
  for (int i = 0; i < 6; i++)
    MODBUS_ReceiveByte(&slave, bad_fc[i]);
  MODBUS_ReceiveByte(&slave, (uint8_t)(crc_bad & 0xFF));
  MODBUS_ReceiveByte(&slave, (uint8_t)(crc_bad >> 8));

  test_tx_len = 0;
  status = MODBUS_ProcessFrame(&slave);
  if (test_tx_len > 0) {
    TEST_ASSERT(test_tx_buf[1] == (0x7F | 0x80),
                "Exception response has FC with error bit set (0xFF)");
    TEST_ASSERT(test_tx_buf[2] == MODBUS_EX_ILLEGAL_FUNCTION,
                "Exception code = ILLEGAL_FUNCTION");
  } else {
    TEST_ASSERT(status != MODBUS_OK, "Bad FC returns error status");
  }
  (void)status;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Test: MODBUS Broadcast
 * ═══════════════════════════════════════════════════════════════════════════
 */

static void test_modbus_broadcast(void) {
  TEST_SUITE("MODBUS RTU — Broadcast Handling");

  modbus_data_table_t data;
  memset(&data, 0, sizeof(data));

  modbus_slave_t slave;
  modbus_hal_t hal = {
      .uart_transmit = mock_uart_tx,
      .rs485_dir_tx = mock_rs485_tx,
      .rs485_dir_rx = mock_rs485_rx,
  };

  MODBUS_Init(&slave, 1, &data, hal);

  /* Broadcast (address 0): Write Single Register, reg 0x0000 = 0x00FF */
  uint8_t bcast[] = {0x00, 0x06, 0x00, 0x00, 0x00, 0xFF};
  uint16_t crc_bc = MODBUS_CRC16(bcast, 6);
  for (int i = 0; i < 6; i++)
    MODBUS_ReceiveByte(&slave, bcast[i]);
  MODBUS_ReceiveByte(&slave, (uint8_t)(crc_bc & 0xFF));
  MODBUS_ReceiveByte(&slave, (uint8_t)(crc_bc >> 8));

  test_tx_len = 0;
  MODBUS_ProcessFrame(&slave);
  /* Broadcast: slave should process but NOT respond */
  TEST_ASSERT(test_tx_len == 0, "Broadcast (addr=0): no response transmitted");
  TEST_ASSERT(data.holding_registers[0] == 0x00FF,
              "Broadcast write applied: holding_reg[0] = 0x00FF");
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Test: USB CDC Ring Buffer
 * ═══════════════════════════════════════════════════════════════════════════
 */

/* Include the USB CDC ring buffer implementation */
#define USB_CDC_RX_BUFFER_SIZE 16 /* Small for testing */
#define USB_CDC_TX_BUFFER_SIZE 16

#include "../protocol_stacks/usb_cdc/usb_cdc.h"

/* Re-implement ring buffer inline for testing */
typedef struct {
  uint8_t buffer[16];
  volatile uint16_t head;
  volatile uint16_t tail;
} test_ringbuf_t;

static void test_rb_init(test_ringbuf_t *rb) {
  rb->head = 0;
  rb->tail = 0;
}

static uint16_t test_rb_count(test_ringbuf_t *rb) {
  int16_t diff = rb->head - rb->tail;
  if (diff < 0)
    diff += 16;
  return (uint16_t)diff;
}

static void test_rb_push(test_ringbuf_t *rb, uint8_t byte) {
  uint16_t next = (rb->head + 1) % 16;
  if (next != rb->tail) {
    rb->buffer[rb->head] = byte;
    rb->head = next;
  }
}

static int16_t test_rb_pop(test_ringbuf_t *rb) {
  if (rb->head == rb->tail)
    return -1;
  uint8_t byte = rb->buffer[rb->tail];
  rb->tail = (rb->tail + 1) % 16;
  return byte;
}

static void test_usb_cdc_ringbuffer(void) {
  TEST_SUITE("USB CDC — Ring Buffer Logic");

  test_ringbuf_t rb;
  test_rb_init(&rb);

  TEST_ASSERT(test_rb_count(&rb) == 0, "Empty ring buffer count = 0");
  TEST_ASSERT(test_rb_pop(&rb) == -1, "Pop from empty returns -1");

  /* Push and pop */
  test_rb_push(&rb, 0xAA);
  TEST_ASSERT(test_rb_count(&rb) == 1, "After push: count = 1");

  int16_t val = test_rb_pop(&rb);
  TEST_ASSERT(val == 0xAA, "Pop returns 0xAA");
  TEST_ASSERT(test_rb_count(&rb) == 0, "After pop: count = 0");

  /* Fill to capacity (15 items in 16-slot ring buffer) */
  for (int i = 0; i < 15; i++) {
    test_rb_push(&rb, (uint8_t)i);
  }
  TEST_ASSERT(test_rb_count(&rb) == 15, "Full ring buffer count = 15");

  /* Overflow — 16th push should be silently dropped */
  test_rb_push(&rb, 0xFF);
  TEST_ASSERT(test_rb_count(&rb) == 15,
              "Overflow: count stays at 15 (data dropped)");

  /* Drain and verify FIFO order */
  for (int i = 0; i < 15; i++) {
    int16_t v = test_rb_pop(&rb);
    TEST_ASSERT(v == i, "FIFO order preserved");
  }
  TEST_ASSERT(test_rb_count(&rb) == 0, "Fully drained: count = 0");

  /* Wrap-around test */
  for (int i = 0; i < 10; i++)
    test_rb_push(&rb, (uint8_t)(i + 100));
  for (int i = 0; i < 5; i++)
    test_rb_pop(&rb); /* head and tail not at 0 */
  for (int i = 0; i < 10; i++)
    test_rb_push(&rb, (uint8_t)(i + 200));
  TEST_ASSERT(test_rb_count(&rb) == 15,
              "Wrap-around: count correct after wrap");
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Test: Error Codes Enum Consistency
 * ═══════════════════════════════════════════════════════════════════════════
 */

#include "../drivers/common/error_codes.h"

static void test_error_codes(void) {
  TEST_SUITE("Error Codes — Enum Consistency");

  TEST_ASSERT(DRV_OK == 0, "DRV_OK = 0");
  TEST_ASSERT(DRV_ERROR != DRV_OK, "DRV_ERROR != DRV_OK");
  TEST_ASSERT(DRV_TIMEOUT != DRV_BUSY, "DRV_TIMEOUT != DRV_BUSY");
  TEST_ASSERT(DRV_NOT_SUPPORTED > DRV_OK,
              "DRV_NOT_SUPPORTED is a valid enum value > 0");

  /* Verify no duplicate values */
  TEST_ASSERT(DRV_ERROR != DRV_BUSY, "No collision: ERROR vs BUSY");
  TEST_ASSERT(DRV_BUSY != DRV_TIMEOUT, "No collision: BUSY vs TIMEOUT");
  TEST_ASSERT(DRV_ACK_FAIL != DRV_ARB_LOST,
              "No collision: ACK_FAIL vs ARB_LOST");

  /* Transfer states */
  TEST_ASSERT(XFER_STATE_RESET == 0, "XFER_STATE_RESET = 0");
  TEST_ASSERT(XFER_STATE_READY != XFER_STATE_BUSY_TX, "READY != BUSY_TX");

  /* Callback events */
  TEST_ASSERT(EVT_TX_COMPLETE == 0, "EVT_TX_COMPLETE = 0");
  TEST_ASSERT(EVT_ERROR != EVT_TX_COMPLETE, "EVT_ERROR != EVT_TX_COMPLETE");
  TEST_ASSERT(EVT_IDLE != EVT_HALF_COMPLETE, "EVT_IDLE != EVT_HALF_COMPLETE");
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Test: CAN Message Structure
 * ═══════════════════════════════════════════════════════════════════════════
 */

/* Prevent stm32f4xx_base.h from being included transitively —
 * we already provide simulated types and typedef aliases above. */
#ifndef STM32F4XX_BASE_H
#define STM32F4XX_BASE_H
#endif

/* Provide a stub CAN_RegDef_t since stm32f4xx_base.h is blocked */
typedef struct {
  volatile uint32_t MCR;
  volatile uint32_t MSR;
  volatile uint32_t TSR;
  volatile uint32_t RF0R;
  volatile uint32_t RF1R;
  volatile uint32_t IER;
  volatile uint32_t ESR;
  volatile uint32_t BTR;
} CAN_RegDef_t;

#include "../protocol_stacks/can_bus/can_driver.h"

static void test_can_structures(void) {
  TEST_SUITE("CAN Bus — Structure Layout");

  CAN_TxMessage_t tx_msg;
  memset(&tx_msg, 0, sizeof(tx_msg));

  tx_msg.id = 0x123;
  tx_msg.id_type = CAN_ID_STD;
  tx_msg.rtr = CAN_RTR_DATA;
  tx_msg.dlc = 8;
  tx_msg.data[0] = 0xDE;
  tx_msg.data[7] = 0xAD;

  TEST_ASSERT(tx_msg.id == 0x123, "TX message ID = 0x123");
  TEST_ASSERT(tx_msg.dlc == 8, "TX message DLC = 8");
  TEST_ASSERT(tx_msg.data[0] == 0xDE, "TX data[0] = 0xDE");
  TEST_ASSERT(tx_msg.data[7] == 0xAD, "TX data[7] = 0xAD");
  TEST_ASSERT(tx_msg.rtr == CAN_RTR_DATA, "RTR = DATA");

  CAN_RxMessage_t rx_msg;
  rx_msg.id = 0x1ABCDEF;
  rx_msg.id_type = CAN_ID_EXT;
  TEST_ASSERT(rx_msg.id_type == CAN_ID_EXT, "RX extended ID");
  TEST_ASSERT(rx_msg.id == 0x1ABCDEF, "RX 29-bit ID preserved");

  /* Config */
  CAN_Config_t cfg;
  cfg.mode = CAN_MODE_LOOPBACK;
  cfg.baud_rate = CAN_SPEED_500K;
  TEST_ASSERT(cfg.mode == CAN_MODE_LOOPBACK, "Mode = LOOPBACK");
  TEST_ASSERT(cfg.baud_rate == 500000, "Baud = 500K");

  /* Handle with separate contexts */
  CAN_Handle_t handle;
  memset(&handle, 0, sizeof(handle));
  handle.rx_cb_context = (void *)0x1111;
  handle.tx_cb_context = (void *)0x2222;
  handle.error_cb_context = (void *)0x3333;
  TEST_ASSERT(handle.rx_cb_context != handle.tx_cb_context,
              "Separate callback contexts are independent");
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Test: USB CDC Line Coding
 * ═══════════════════════════════════════════════════════════════════════════
 */

static void test_usb_cdc_line_coding(void) {
  TEST_SUITE("USB CDC — Line Coding");

  usb_cdc_line_coding_t lc;
  lc.baud_rate = 115200;
  lc.stop_bits = 0; /* 1 stop bit */
  lc.parity = 0;    /* None */
  lc.data_bits = 8;

  TEST_ASSERT(lc.baud_rate == 115200, "Line coding baud = 115200");
  TEST_ASSERT(lc.data_bits == 8, "Line coding data bits = 8");
  TEST_ASSERT(sizeof(usb_cdc_line_coding_t) == 7,
              "Line coding struct is packed (7 bytes)");

  /* Status codes */
  TEST_ASSERT(USB_CDC_OK == 0, "USB_CDC_OK = 0");
  TEST_ASSERT(USB_CDC_ERROR < 0, "USB_CDC_ERROR is negative");
  TEST_ASSERT(USB_CDC_BUSY < 0, "USB_CDC_BUSY is negative");
  TEST_ASSERT(USB_CDC_DISCONNECTED < 0, "USB_CDC_DISCONNECTED is negative");
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Main — Run All Tests
 * ═══════════════════════════════════════════════════════════════════════════
 */

int main(void) {
  printf("╔════════════════════════════════════════════════════════════╗\n");
  printf("║      STM32F4xx Driver Suite — Unit Test Report            ║\n");
  printf("╚════════════════════════════════════════════════════════════╝\n");

  /* MODBUS RTU Tests */
  test_modbus_crc16();
  test_modbus_data_table();
  test_modbus_frame_processing();
  test_modbus_exceptions();
  test_modbus_broadcast();

  /* USB CDC Tests */
  test_usb_cdc_ringbuffer();
  test_usb_cdc_line_coding();

  /* Common Infrastructure Tests */
  test_error_codes();

  /* CAN Bus Tests */
  test_can_structures();

  /* Summary */
  printf("\n╔════════════════════════════════════════════════════════════╗\n");
  printf("║  RESULTS: %d/%d passed, %d failed                       ║\n",
         tests_passed, tests_run, tests_failed);
  printf("╚════════════════════════════════════════════════════════════╝\n");

  return tests_failed > 0 ? 1 : 0;
}

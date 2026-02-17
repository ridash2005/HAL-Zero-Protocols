# 📡 Communication Protocol Stacks

> **Production-quality protocol implementations** for industrial automation, automotive, and PC-MCU connectivity.  
> Built on top of the bare-metal peripheral drivers in `../drivers/`.

---

## 📂 Directory Structure

```
protocol_stacks/
├── README.md               ← You are here
├── modbus/
│   ├── modbus_rtu.h        ← Full Modbus RTU slave responder
│   └── modbus_rtu.c
├── can_bus/
│   ├── can_driver.h        ← CAN 2.0B controller driver
│   └── can_driver.c
└── usb_cdc/
    ├── usb_cdc.h            ← USB CDC ACM virtual COM port
    └── usb_cdc.c
```

---

## ✅ Protocol Status

| Protocol | Standard | Physical Layer | Status |
|----------|----------|----------------|--------|
| **MODBUS RTU** | Modbus App Protocol V1.1b3 | RS-485 (via UART) | Complete |
| **CAN Bus** | CAN 2.0B (Bosch) | CAN transceiver | Complete |
| **USB CDC** | USB 2.0 CDC ACM | USB Full-Speed | Complete |

---

## 📐 Architecture

```
┌─────────────────────────────────────────┐
│          Application Layer              │
├─────────┬───────────┬───────────────────┤
│ MODBUS  │ CAN Node  │  USB CDC Device   │
│  RTU    │ Firmware  │  (Virtual COM)    │
├─────────┼───────────┼───────────────────┤
│  UART   │    CAN    │    USB OTG FS     │
│ driver  │  driver   │    registers      │
├─────────┴───────────┴───────────────────┤
│    Peripheral Drivers (../drivers/)     │
├─────────────────────────────────────────┤
│      STM32F4xx Hardware (Cortex-M4F)    │
└─────────────────────────────────────────┘
```

---

## 🏭 MODBUS RTU

**Industry:** PLC communication, SCADA systems, industrial sensors

### Supported Function Codes

| Code | Name | Access |
|------|------|--------|
| `0x01` | Read Coils | R |
| `0x02` | Read Discrete Inputs | R |
| `0x03` | Read Holding Registers | R |
| `0x04` | Read Input Registers | R |
| `0x05` | Write Single Coil | W |
| `0x06` | Write Single Register | W |
| `0x0F` | Write Multiple Coils | W |
| `0x10` | Write Multiple Registers | W |

### Key Features
- CRC-16/MODBUS validation (polynomial 0xA001)
- Exception response generation (illegal function, address, value)
- Configurable data tables (coils, discrete inputs, holding/input registers)
- RS-485 direction control callbacks (TX/RX switching)
- Broadcast address (0) support (no response)

### Quick Start
```c
#include "protocol_stacks/modbus/modbus_rtu.h"

modbus_data_table_t data;
modbus_slave_t slave;

modbus_hal_t hal = {
    .uart_transmit = my_uart_send,
    .rs485_dir_tx  = my_rs485_tx_enable,
    .rs485_dir_rx  = my_rs485_rx_enable,
};

MODBUS_Init(&slave, 1, &data, hal);   /* Slave address = 1 */

/* In UART RX ISR: */
MODBUS_ReceiveByte(&slave, received_byte);

/* After 3.5-char silence detected: */
MODBUS_ProcessFrame(&slave);
```

---

## 🚗 CAN Bus

**Industry:** Automotive ECUs, vehicle diagnostics (OBD-II), industrial machinery

### Key Features
- CAN 2.0B (standard and extended identifiers)
- Hardware filter configuration (mask/list mode)
- TX mailbox management with priority arbitration
- RX FIFO handling (FIFO 0 and FIFO 1)
- Loopback and silent mode for diagnostics
- Error detection and bus-off recovery

---

## 🔌 USB CDC

**Industry:** PC ↔ MCU communication, debug consoles, data acquisition

### Key Features
- USB Full-Speed device mode (OTG FS peripheral)
- CDC ACM class — appears as virtual COM port on host
- Configurable VID/PID and string descriptors
- Interrupt-driven EP0 control transfers
- Bulk IN/OUT endpoints for data exchange

---

## 🎯 Design Principles

1. **Self-Contained** — Each protocol stack is independent and includes its own header/implementation
2. **HAL Abstraction** — Hardware access is abstracted via callback structures for portability
3. **Spec-Compliant** — Implementations follow the official protocol specifications
4. **Production-Ready** — Error handling, bounds checking, and edge cases are addressed

---

*These implementations demonstrate the kind of protocol-level firmware that embedded engineers build in companies like Bosch, Siemens, and Texas Instruments.*

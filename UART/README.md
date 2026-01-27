# UART (Universal Asynchronous Receiver-Transmitter)

UART is one of the oldest and most common communication protocols in embedded systems. Unlike SPI or I2C, it is **asynchronous**, meaning there is no shared clock signal. Instead, both devices must agree on a timing speed (baud rate) beforehand.

## 📌 Key Characteristics

*   **Type**: Asynchronous Serial
*   **Wires**: 2 Signal Wires (RX, TX) + 1 Common Ground (GND)
*   **Duplex**: Full Duplex (Can transmit and receive simultaneously)
*   **Distance**: Short (TTL Logic), Long (with RS-232/RS-485 definitions)


## 🔌 Wiring Connection (Architecture)

The most common mistake beginners make is wiring TX to TX. **Always cross the lines!**

```mermaid
graph LR
    subgraph Device_A ["Device A (Master/PC)"]
        direction TB
        TX_A[TX Pin]
        RX_A[RX Pin]
        GND_A[GND]
    end
    
    subgraph Device_B ["Device B (Slave/Sensor)"]
        direction TB
        TX_B[TX Pin]
        RX_B[RX Pin]
        GND_B[GND]
    end

    TX_A -- Data Flow --> RX_B
    TX_B -- Data Flow --> RX_A
    GND_A == Common Ground == GND_B

    style TX_A fill:#ff9999,stroke:#333,stroke-width:2px
    style RX_B fill:#99ff99,stroke:#333,stroke-width:2px
    style TX_B fill:#ff9999,stroke:#333,stroke-width:2px
    style RX_A fill:#99ff99,stroke:#333,stroke-width:2px
```

| Device A | Connection | Device B |
| :--- | :---: | :--- |
| **TX** (Transmit) | &rarr; | **RX** (Receive) |
| **RX** (Receive) | &larr; | **TX** (Transmit) |
| **GND** | &harr; | **GND** |

> **Note**: Both devices must share a common ground reference.

## ⏱ Frame Format

A single "packet" or frame in UART usually consists of 10 bits in total (for 8-N-1 config):

1.  **Idle State**: Line is High (Voltage level).
2.  **Start Bit**: 1 bit (Line goes Low). Signals the receiver that data is coming.
3.  **Data Bits**: 5 to 9 bits (usually 8). Least Significant Bit (LSB) first.
4.  **Parity Bit**: (Optional) For error checking. Can be None, Even, or Odd.
5.  **Stop Bit**: 1 or 2 bits (Line goes High). Ensures the line returns to Idle state.

### visual representation
```text
      Idle    Start   D0  D1  D2  D3  D4  D5  D6  D7   Stop   Idle
Line: 1111111 0       1   0   1   0   1   1   0   1    1      1111...
```

## ⚙️ Configuration (The "8-N-1" Standard)
Most devices use the **8-N-1** configuration:
*   **8** Data bits
*   **N**o Parity
*   **1** Stop bit

**Baud Rate**: The speed of datas in bits per second (bps). Common values:
*   9600 (Standard for simple modules like GPS)
*   115200 (Standard for fast debug logs/firmware upload)

## ⚠️ Common Pitfalls
1.  **Baud Rate Mismatch**: If you see garbage characters (e.g., `?&*`), your baud rates likely don't match.
2.  **Floating Pins**: If RX is disconnected, it might read random noise.
3.  **Voltage Levels**:
    *   **TTL (5V/3.3V)**: Used by microcontrollers (Arduino, STM32).
    *   **RS-232 (+/- 12V)**: Used by old PC serial ports. **Do not connect directly to a microcontroller!** You will fry it.

## 💻 Examples provided

Check the `examples/` folder for:
1.  **Arduino**: A simple loopback and command parser.
3.  **FPGA (SystemVerilog)**: synthesized UART TX module with **Parity Generation** (Even/Odd) and active Busy/Error detection.
4.  **Python**: Scripts to talk to your microcontroller from a PC.

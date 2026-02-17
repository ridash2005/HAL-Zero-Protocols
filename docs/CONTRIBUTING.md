# Contributing Guidelines

Thank you for your interest in contributing to the **Communication Protocols** repository!

We want this to be the best open-source resource for embedded communication protocols. To ensure quality, please follow these guidelines.

## 🛠 How to Contribute

1.  **Fork the Repository**: Click the "Fork" button on GitHub.
2.  **Clone your Fork**: `git clone https://github.com/YOUR_USERNAME/Communication_Protocols.git`
3.  **Create a Branch**: `git checkout -b feature/new-protocol` or `fix/typo-fix`.
4.  **Make Changes**: Add your code or documentation.
5.  **Test**: Ensure your code compiles (for Arduino) or runs (for Python).
6.  **Commit**: Use descriptive commit messages.
7.  **Push**: `git push origin feature/new-protocol`.
8.  **Open a Pull Request**: Submit your PR on the main repository.

## 📝 Style Guide

### Code
*   **Comments**: Explain *why*, not just *what*. Beginners count on this!
*   **Variable Names**: Use `sensorValue` instead of `val`.
*   **Format**:
    *   Arduino: Standard K&R style.
    *   Python: PEP 8.

### Documentation
*   Use Markdown headers (`#`, `##`, `###`) to structure content.
*   Include diagrams (ASCII or images) where possible.
*   Check spelling and grammar.

## 🚧 Roadmap & Help Wanted

We are looking for contributions in the following areas. If you want to work on one of these, please check the Issues tab to avoid duplication!

### 1. 📡 Missing Protocols
We need comprehensive guides and examples for:
*   **RS-485**: Robust industrial serial (physical layer for MODBUS).
*   **1-Wire**: For sensors like DS18B20.
*   **I2S**: Audio data transmission.
*   **Modbus TCP**: TCP/IP variant of Modbus (RTU is already implemented).

> **Already Implemented:** CAN Bus (`protocol_stacks/can_bus/`), MODBUS RTU (`protocol_stacks/modbus/`), USB CDC (`protocol_stacks/usb_cdc/`).

### 2. 💻 New Platforms
Expand our coverage beyond Arduino/STM32/FPGA:
*   **ESP32**: implementations using **ESP-IDF** (FreeRTOS based).
*   **Raspberry Pi Pico (RP2040)**: using the **C/C++ SDK**.
*   **Rust Embedded**: Examples using `embedded-hal` or `embassy`.

### 3. ⚡ Advanced Features
*   **Hardware-in-loop testing**: Automated test scripts that validate real hardware behavior.
*   **Self-checking testbenches**: SystemVerilog testbenches with `$assert` / `$error` for the FPGA modules.
*   **Unit Tests**: Host-compiled unit tests for the bare-metal drivers (see `verification/`).

### 4. 📚 Documentation
*   Add more interactive timing diagrams (WaveDrom).
*   Translations into other languages.

Happy Coding!

# Verification Suite

This directory contains verification scripts to ensure the correctness of the Communication Protocols driver suite.

## 1. Python Mock Verification (`mock_verify.py`)
This suite uses `unittest.mock` to simulate hardware behavior and verify the logic of the Python-based tools and prototypes.
- **Dependencies**: Python 3
- **Run**: `python mock_verify.py`

## 2. Host-Based C Unit Tests (`test_drivers.c`)
This compiles the actual driver C code against simulated register definitions (`stm32f4xx_base.h` in simulation mode) to verify bit-manipulation logic, buffer handling, and protocol state machines on your PC.
- **Dependencies**: C Compiler (MSVC `cl`, GCC, or Clang)
- **Run (Windows MSVC)**: `run_verification.bat`
- **Run (GCC/Linux)**: `gcc -std=c11 -DUNIT_TEST -I../drivers/common -I../protocol_stacks/modbus -I../protocol_stacks/usb_cdc -I../protocol_stacks/can_bus test_drivers.c -o test_drivers && ./test_drivers`

## 3. How to Run All Tests
For Windows users with the Visual Studio "Native Tools Command Prompt":
```cmd
verification\run_verification.bat
```
This script will:
1. Execute the Python mock tests.
2. Compile the C driver code with MSVC.
3. specific defines and include paths.
4. Run the resulting executable to verify C logic.

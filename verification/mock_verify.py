import sys
from unittest.mock import MagicMock

# Mock smbus
sys.modules['smbus'] = MagicMock()
import smbus
mock_bus = smbus.SMBus.return_value
mock_bus.read_byte_data.return_value = 0x55

# Mock spidev
sys.modules['spidev'] = MagicMock()
import spidev
mock_spi = spidev.SpiDev.return_value
mock_spi.xfer2.side_effect = lambda x: x # Echo back

# Mock serial
sys.modules['serial'] = MagicMock()
import serial
mock_ser = serial.Serial.return_value
mock_ser.is_open = True
mock_ser.in_waiting = 5
mock_ser.readline.return_value = b"Mock Response\n"

def run_script(path):
    with open(path, 'r') as f:
        code = f.read()
    # Execute in a custom namespace
    namespace = {'__name__': '__main__', 'input': MagicMock(side_effect=['hello', 'exit']), 'time': MagicMock()}
    exec(code, namespace)

def test_i2c():
    print("\nDRY RUN: Python I2C Test")
    try:
        run_script('I2C/examples/python/smbus_test.py')
        print("Logic Check: PASSED")
    except Exception as e:
        print(f"Logic Check: FAILED with {e}")

def test_uart():
    print("\nDRY RUN: Python UART Test")
    try:
        run_script('UART/examples/python/uart_sender.py')
        print("Logic Check: PASSED")
    except Exception as e:
        print(f"Logic Check: FAILED with {e}")

if __name__ == "__main__":
    test_i2c()
    test_uart()

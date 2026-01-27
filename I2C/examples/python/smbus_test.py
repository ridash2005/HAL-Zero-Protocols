import smbus
import time
import sys

# Requirements:
# - Raspberry Pi (or Linux board) with I2C enabled
# - Library: pip install smbus2

# Bus ID: usually 1 for newer Pis, 0 for very old ones
I2C_BUS_ID = 1

# Example Device Address (Change this to match your sensor!)
# Run 'i2cdetect -y 1' in terminal to find your address
DEVICE_ADDRESS = 0x68 # Common for RTCs like DS3231 or GY-521 IMU

def main():
    print("--- Python I2C Test (SMBus) ---")
    
    try:
        # Create an SMBus object
        bus = smbus.SMBus(I2C_BUS_ID)
    except FileNotFoundError:
        print("Error: I2C bus not found. Please enable I2C interface.")
        print("On Raspberry Pi: sudo raspi-config -> Interface Options -> I2C")
        sys.exit(1)
    except PermissionError:
        print("Error: Permission denied. Try running with 'sudo'.")
        sys.exit(1)

    print(f"Connected to Bus {I2C_BUS_ID}")
    print(f"Attempting to read from device at 0x{DEVICE_ADDRESS:02X}...")
    
    try:
        # Simple test: Read 1 byte from register 0
        # Most sensors allow reading register 0 to check ID or status
        val = bus.read_byte_data(DEVICE_ADDRESS, 0x00)
        
        print(f"Success! Read value: {val} (0x{val:02X})")
        
    except OSError as e:
        print(f"\nCommunication Error: {e}")
        print(f"1. Is the device connected?")
        print(f"2. Is the address 0x{DEVICE_ADDRESS:02X} correct?")
        print(f"3. Are pull-up resistors installed?")
        
    finally:
        bus.close()

if __name__ == "__main__":
    main()

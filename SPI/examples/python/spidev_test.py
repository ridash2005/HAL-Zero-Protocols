import spidev
import time
import sys

# Requirements:
# - Linux environment (e.g., Raspberry Pi) with SPI enabled in raspi-config
# - Library: pip install spidev

def main():
    print("--- Python SPI Test (spidev) ---")
    
    # Create SPI object
    spi = spidev.SpiDev()
    
    # Open Bus 0, Device 0 (CE0 pin)
    # RPi has: /dev/spidev0.0 and /dev/spidev0.1
    try:
        spi.open(0, 0)
    except FileNotFoundError:
        print("Error: SPI device not found. Are you on a Raspberry Pi with SPI enabled?")
        sys.exit(1)
        
    # Configuration
    spi.max_speed_hz = 500000  # 500 kHz
    spi.mode = 0b00            # Mode 0 (CPOL=0, CPHA=0)
    
    print(f"SPI Opened. Mode: {spi.mode}, Speed: {spi.max_speed_hz} Hz")
    
    try:
        while True:
            # Data to send (list of integers)
            to_send = [0x01, 0x02, 0xFF]
            
            # xfer2 maintains CS active during efficient bursts
            received = spi.xfer2(to_send)
            
            print(f"Sent: {to_send}")
            print(f"Recv: {received}")
            print("-" * 20)
            
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        spi.close()

if __name__ == "__main__":
    main()

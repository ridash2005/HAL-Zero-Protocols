import serial
import time
import sys

# Configuration
# Windows Example: 'COM3', 'COM4'
# Linux/Mac Example: '/dev/ttyUSB0', '/dev/ttyACM0'
PORT = 'COM3' 
BAUD_RATE = 9600

def main():
    print("--- Python UART Sender ---")
    
    try:
        # Initialize Serial Port
        # timeout=1 means read() will wait 1s max
        ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
        
        # Allow Arduino reset time if DTR is triggered
        time.sleep(2) 
        
        print(f"Connected to {PORT} at {BAUD_RATE} baud.")
        print("Type 'exit' to quit.\n")
        
        while True:
            user_input = input("Send: ")
            
            if user_input.lower() == 'exit':
                break
            
            # Send data (Need to encode string to bytes)
            # We add a newline character because many parsers expect it
            ser.write((user_input + '\n').encode('utf-8'))
            
            # Brief pause to let the device process
            time.sleep(0.1)
            
            # Check if there is data waiting to be read
            if ser.in_waiting > 0:
                # Read line, decode bytes to string, strip whitespace
                response = ser.readline().decode('utf-8').strip()
                print(f"Received: {response}")
                
    except serial.SerialException as e:
        print(f"\nError opening serial port {PORT}: {e}")
        print("Check if the device is plugged in and the PORT variable is correct.")
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    main()

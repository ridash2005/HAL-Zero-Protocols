/*
 * I2C Scanner
 *
 * This utility sketch scans the I2C bus for active devices.
 * It attempts to communicate with every address from 0x01 to 0x7F.
 * If a device sends an ACK (Acknowledge), it is detected.
 *
 * Hardware Required:
 * - Arduino Board
 * - Any I2C Device (Sensor, Display, etc.)
 * - Pull-up resistors on SDA/SCL (usually included on breakout boards)
 */

#include <Wire.h>

void setup() {
  Wire.begin(); // Initialize I2C as Master

  Serial.begin(9600);
  while (!Serial)
    ; // Wait for Serial Monitor

  Serial.println("\n--- I2C Scanner ---");
}

void loop() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++) {
    // The simple trick: Try to start a transmission.
    // If the device exists, it will ACK, and endTransmission returns 0.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("Scan complete.\n");

  delay(5000); // Wait 5 seconds before next scan
}

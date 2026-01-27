/*
 * SPI Master Loopback Example
 *
 * This sketch demonstrates rudimentary SPI usage.
 *
 * Hardware Setup:
 * Connect MOSI directly to MISO to create a loopback.
 * - Arduino Uno: Pin 11 to Pin 12
 * - Arduino Mega: Pin 51 to Pin 50
 * - ICSP Header: MOSI to MISO
 */

#include <SPI.h>

const int CS_PIN = 10; // Standard Chip Select pin on Uno

void setup() {
  Serial.begin(9600);
  while (!Serial)
    ;

  // Initialize SPI hardware
  SPI.begin();

  // Configure Chip Select
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH); // High means "Deselected"

  Serial.println("--- SPI Loopback Test ---");
  Serial.println("Connect MOSI to MISO to see matching Send/Recv values.");
}

void loop() {
  byte valueToSend = 0x42; // 'B'

  // Configure SPI parameters: 14MHz max, MSB First, Mode 0
  // Always wrap transfers in begin/endTransaction for best practice
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

  // 1. Select the device
  digitalWrite(CS_PIN, LOW);

  // 2. Transmit and Receive simultaneously
  // In SPI, for every byte you shift out, a byte is shifted in.
  byte receivedValue = SPI.transfer(valueToSend);

  // 3. Deselect the device
  digitalWrite(CS_PIN, HIGH);

  SPI.endTransaction();

  Serial.print("Sent: 0x");
  Serial.print(valueToSend, HEX);
  Serial.print(" -> Received: 0x");
  Serial.println(receivedValue, HEX);

  delay(1000);
}

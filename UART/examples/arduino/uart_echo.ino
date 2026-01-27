/*
 * UART Echo Example
 * This sketch listens for incoming characters on the Serial port and echoes them back.
 * Configuration: 9600 Baud, 8-N-1
 * 
 * Hardware Required:
 * - Any Arduino board (Uno, Nano, Mega, etc.)
 * - USB cable
 */

void setup() {
  // Initialize Serial at 9600 bits per second
  Serial.begin(9600);
  
  // Wait for serial port to connect (needed for native USB boards like Leonardo/Micro)
  while (!Serial) {
    ; 
  }

  Serial.println("UART Echo Ready! Type something...");
}

void loop() {
  // Check if data is available to read
  if (Serial.available() > 0) {
    // Read the incoming byte
    char incomingByte = Serial.read();

    // Echo it back to the computer/sender
    Serial.print("I received: ");
    Serial.println(incomingByte);
  }
}

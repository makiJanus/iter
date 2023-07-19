#include <Wire.h>   // Library for I2C communication
#include <Servo.h>  // Library for servo control

#define I2C_SLAVE_ADDRESS 8  // I2C address for the Arduino board acting as a slave

Servo servo1;  // Create an instance of the Servo class called servo1

void setup() {
  Wire.begin(I2C_SLAVE_ADDRESS);      // Start I2C communication as a slave with the specified address
  Wire.onReceive(receiveEvent);       // Register the callback function receiveEvent() for the I2C receive event

  servo1.attach(7);   // Connect servo1 to pin 7

  servo1.write(80);   // Set the initial position of servo1 to 80 degrees
}

void loop() {
  // Servo control code (if any)
}

void receiveEvent(int numBytes) {
  // This function is called automatically when data is received via I2C

  while (Wire.available() >= 3) {
    // Continue reading data as long as there are at least 3 bytes available in the buffer

    int servoIndex = Wire.read();       // Read the servo index (the first byte)
    int angleHighByte = Wire.read();    // Read the high byte of the angle (the second byte)
    int angleLowByte = Wire.read();     // Read the low byte of the angle (the third byte)

    int angle = (angleHighByte << 8) | angleLowByte;
    // Combine the high and low bytes to form a 16-bit angle value

    if (servoIndex == 1)
      servo1.write(angle);
    // If the received servo index is 1, set the angle of servo1 to the received value
  }
}

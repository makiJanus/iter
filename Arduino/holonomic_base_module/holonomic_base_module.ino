// Arduino nano pinout - https://www.studiopieters.nl/arduino-nano-pinout/
// Pin change interrupts configuration - https://thewanderingengineer.com/2014/08/11/arduino-pin-change-interrupts/

#include <avr/interrupt.h>  // Library for interrupt handling
#include <Wire.h>           // Library for I2C communication

// I2C Pin Definition
#define I2C_SLAVE_ADDRESS 9  // I2C address for the Arduino board acting as a slave
float receivedData[3];       // Array to store received data

// Pin Definitions for Encoders
#define encoderAPinA 2    // Encoder A Pin A
#define encoderAPinB 4    // Encoder A Pin B
#define encoderBPinA 7    // Encoder B Pin A
#define encoderBPinB 8    // Encoder B Pin B
#define encoderCPinA A0   // Encoder C Pin A
#define encoderCPinB A1   // Encoder C Pin B

// Pin Definitions for H-bridge Motors
#define motorCPinA 10   // Motor C Pin A
#define motorCPinB 11   // Motor C Pin B
#define motorAPinA 3    // Motor A Pin A
#define motorAPinB 9    // Motor A Pin B
#define motorBPinA 5    // Motor B Pin A
#define motorBPinB 6    // Motor B Pin B

// Encoder Variables
volatile int encoderACount = 0;  // Encoder A count
volatile int encoderBCount = 0;  // Encoder B count
volatile int encoderCCount = 0;  // Encoder C count

// Wheel Parameters
#define wheelPerimeter 8.2212      // Wheel perimeter in cm
#define countsPerRevolution 12500  // Approximate counts per revolution
float linear_vel_a = 0;            // Linear velocity of motor A in cm/s
float linear_vel_b = 0;            // Linear velocity of motor B in cm/s
float linear_vel_c = 0;            // Linear velocity of motor C in cm/s

// Time Variables
unsigned long previousTime = 0;    // Previous time value
float elapsedTime = 0;             // Elapsed time since the previous loop

// PID Variables
float Kp = 3.0;                    // Proportional constant
float Ki = 0.01;                   // Integral constant
float Kd = 0.01;                   // Derivative constant

float a_error = 0.0;               // Error for motor A
float a_lastError = 0.0;           // Previous error for motor A
float a_integral = 0.0;            // Integral term for motor A
float a_derivative = 0.0;          // Derivative term for motor A
float a_output = 0.0;              // PID output for motor A

float b_error = 0.0;               // Error for motor B
float b_lastError = 0.0;           // Previous error for motor B
float b_integral = 0.0;            // Integral term for motor B
float b_derivative = 0.0;          // Derivative term for motor B
float b_output = 0.0;              // PID output for motor B

float c_error = 0.0;               // Error for motor C
float c_lastError = 0.0;           // Previous error for motor C
float c_integral = 0.0;            // Integral term for motor C
float c_derivative = 0.0;          // Derivative term for motor C
float c_output = 0.0;              // PID output for motor C

// PID control variables
float a_desiredVel = 0.0;          // Desired velocity in cm/s for motor A
float b_desiredVel = 0.0;          // Desired velocity in cm/s for motor B
float c_desiredVel = 0.0;          // Desired velocity in cm/s for motor C

int velocity_signal_limiter = 150; // Velocity signal limiter in cm/s

// Function to handle external interrupt for Encoder A
void handleEncoderA()
{
  if (digitalRead(encoderAPinA) == digitalRead(encoderAPinB))
    encoderACount++;
  else
    encoderACount--;
}

// Function to handle pin change interrupt for Encoder B
ISR(PCINT2_vect)
{
  if (digitalRead(encoderBPinA) == digitalRead(encoderBPinB))
    encoderBCount++;
  else
    encoderBCount--;
}

// Function to handle pin change interrupt for Encoder C
ISR(PCINT1_vect)
{
  if (digitalRead(encoderCPinA) == digitalRead(encoderCPinB))
    encoderCCount++;
  else
    encoderCCount--;
}

// Function to control the H-bridge motor
void h_bridge_motor(int A, int B, bool forward, int digital_control)
{
  if(forward)
  {
    digitalWrite(B, LOW);
    analogWrite(A, digital_control);
  }
  else
  {
    digitalWrite(A, LOW);
    analogWrite(B, digital_control);
  }
}

// Callback function for receiving data
void receiveData(int byteCount)
{
  byte first_byte = Wire.read();  // Read the first byte
  byteCount = Wire.available();   // Get the number of available bytes

  if (byteCount == sizeof(receivedData))
  {
    Wire.readBytes((byte*)receivedData, byteCount);  // Read the received bytes into the receivedData array

    // Process received floats
    float value1 = receivedData[0];
    float value2 = receivedData[1];
    float value3 = receivedData[2];

    a_desiredVel = value1;
    b_desiredVel = value2;
    c_desiredVel = value3;

    // Print received values
    // Serial.print("Received values: ");
    // Serial.print(value1);
    // Serial.print(", ");
    // Serial.print(value2);
    // Serial.print(", ");
    // Serial.println(value3);
  }
}

void setup()
{
  // Serial.begin(9600);

  // Wire Configuration
  Wire.begin(I2C_SLAVE_ADDRESS);      // Start I2C communication as a slave with specified address
  Wire.onReceive(receiveData);        // Register callback for I2C receive event

  // Set Encoder Pins as INPUT_PULLUP
  pinMode(encoderAPinA, INPUT_PULLUP);
  pinMode(encoderAPinB, INPUT_PULLUP);
  pinMode(encoderBPinA, INPUT_PULLUP);
  pinMode(encoderBPinB, INPUT_PULLUP);
  pinMode(encoderCPinA, INPUT_PULLUP);
  pinMode(encoderCPinB, INPUT_PULLUP);

  // Set H-bridge Motor Pins as OUTPUT
  pinMode(motorAPinA, OUTPUT);
  pinMode(motorAPinB, OUTPUT);
  pinMode(motorBPinA, OUTPUT);
  pinMode(motorBPinB, OUTPUT);
  pinMode(motorCPinA, OUTPUT);
  pinMode(motorCPinB, OUTPUT);

  // Turn off motor pins initially
  digitalWrite(motorAPinA, 0);
  digitalWrite(motorAPinB, 0);
  digitalWrite(motorBPinA, 0);
  digitalWrite(motorBPinB, 0);
  digitalWrite(motorCPinA, 0);
  digitalWrite(motorCPinB, 0);

  // Attach Interrupt for Encoder A
  attachInterrupt(digitalPinToInterrupt(encoderAPinA), handleEncoderA, CHANGE);

  // Handling pin change interrupts
  cli();                              // Disable interrupts
  PCICR  |= 0b00000110;                // Enable pin change interrupt for Port B and D
  PCMSK1 |= 0b00000001;                // Enable pin change interrupt for PCINT8 (A0 on Arduino)
  PCMSK2 |= 0b10000000;                // Enable pin change interrupt for PCINT23 (D7 on Arduino)
  sei();                              // Enable interrupts

  // Initialize time to measure wheel velocities
  previousTime = millis();
}

void loop()
{
  // Calculate linear velocity in cm/s every 20 ms or so
  if(millis() - previousTime >= 20)
  {
    elapsedTime = millis() - previousTime;
    linear_vel_a = encoderACount * wheelPerimeter * 1000 / (countsPerRevolution * elapsedTime);
    linear_vel_b = encoderBCount * wheelPerimeter * 1000 / (countsPerRevolution * elapsedTime);
    linear_vel_c = encoderCCount * wheelPerimeter * 1000 / (countsPerRevolution * elapsedTime);
    encoderACount = 0;
    encoderBCount = 0;
    encoderCCount = 0;
    previousTime = millis();

    // Calculate PID output for motor A
    a_error = a_desiredVel - linear_vel_a;                         // Calculate the velocity error
    a_integral += (a_error * elapsedTime);                         // Calculate the integral term
    a_derivative = (a_error - a_lastError) / elapsedTime;          // Calculate the derivative term
    a_output = (Kp * a_error) + (Ki * a_integral) + (Kd * a_derivative);  // Calculate the PID output

    // Adjust motor velocity based on the PID output
    int a_motorSpeed = abs(a_output);                              // Take the absolute value for motor speed
    if (a_motorSpeed > velocity_signal_limiter) a_motorSpeed = velocity_signal_limiter;
    bool a_motorDirection = (a_output >= 0);                       // Determine the motor direction (true for forward, false for backward)

    // Apply the motor speed and direction to control the motor
    h_bridge_motor(motorAPinA, motorAPinB, a_motorDirection, a_motorSpeed);

    // Calculate PID output for motor B
    b_error = b_desiredVel - linear_vel_b;                         // Calculate the velocity error
    b_integral += (b_error * elapsedTime);                         // Calculate the integral term
    b_derivative = (b_error - b_lastError) / elapsedTime;          // Calculate the derivative term
    b_output = (Kp * b_error) + (Ki * b_integral) + (Kd * b_derivative);  // Calculate the PID output

    // Adjust motor velocity based on the PID output
    int b_motorSpeed = abs(b_output);                              // Take the absolute value for motor speed
    if (b_motorSpeed > velocity_signal_limiter) b_motorSpeed = velocity_signal_limiter;
    bool b_motorDirection = (b_output >= 0);                       // Determine the motor direction (true for forward, false for backward)

    // Apply the motor speed and direction to control the motor
    h_bridge_motor(motorBPinA, motorBPinB, b_motorDirection, b_motorSpeed);

    // Calculate PID output for motor C
    c_error = c_desiredVel - linear_vel_c;                         // Calculate the velocity error
    c_integral += (c_error * elapsedTime);                         // Calculate the integral term
    c_derivative = (c_error - c_lastError) / elapsedTime;          // Calculate the derivative term
    c_output = (Kp * c_error) + (Ki * c_integral) + (Kd * c_derivative);  // Calculate the PID output

    // Adjust motor velocity based on the PID output
    int c_motorSpeed = abs(c_output);                              // Take the absolute value for motor speed
    if (c_motorSpeed > velocity_signal_limiter) c_motorSpeed = velocity_signal_limiter;
    bool c_motorDirection = (c_output >= 0);                       // Determine the motor direction (true for forward, false for backward)

    // Apply the motor speed and direction to control the motor
    h_bridge_motor(motorCPinA, motorCPinB, c_motorDirection, c_motorSpeed);
  }
}
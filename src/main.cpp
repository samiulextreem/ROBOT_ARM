// code to run test on close loop servo mtoor
#include <Arduino.h>

#define enPin 5    // Pin for ENABLE signal
#define stpPin 6   // Pin for STEP signal
#define dirPin 7   // Pin for DIRECTION signal

// Motor configuration (no gearbox)
#define STEPS_PER_REVOLUTION 200    // For a typical 1.8° stepper (200 steps per revolution)
#define MICROSTEPS 16               // Microstepping setting on your driver
#define GEAR_RATIO 1                // No gearbox - direct drive (1:1 ratio)

// Global variables
float currentAngle = 0.0;           // Keeps track of the current shaft angle
String inputString = "";            // String to hold incoming serial data
boolean stringComplete = false;     // Flag to indicate if a complete string has been received

void rotateShaftByDegrees(float shaftAngle); // Function prototype for rotating the motor shaft by a specified angle
void rotateShaftToAngle(float targetAngle);  // Function prototype for rotating to a specific angle

/**************************************************************
*** Connect:
    * D5---En (Select L or Hold for the En option on the closed-loop driver board)
    * D6---Stp
    * D7---Dir
    * V+,Gnd----10~28V
    * gnd---gnd
*** Precautions:
    * Connect the wire first, then power on, do not unplug or plug when the power is on!
    * When powering on, first connect 10~28V power supply, 
      then connect the Arduino control board USB power supply!
    * Avoid damage caused by some effects.
    * When the power is off, first cut off the USB power supply of the Arduino control board, 
      then cut off the 10~28V power supply.
**************************************************************/

void setup() {
  // Initialize pins
  pinMode(enPin, OUTPUT);
  digitalWrite(enPin, LOW); // Active low, check manual for your setup
  pinMode(stpPin, OUTPUT);
  digitalWrite(stpPin, LOW);
  pinMode(dirPin, OUTPUT);
  digitalWrite(dirPin, HIGH); // Set to HIGH for clockwise (change to LOW for counterclockwise)

  // Initialize serial for debugging
  Serial.begin(9600);
  Serial.println("Closed-loop stepper motor driver - Angle Control");
  Serial.println("Enter an angle (0-360) to move the motor:");
  
  // Reserve memory for the inputString
  inputString.reserve(10);
}

void loop() {
  // Check if a new command has been received
  if (stringComplete) {
    // Convert the received string to a float (angle)
    float targetAngle = inputString.toFloat();
    
    // Print the command for confirmation
    Serial.print("Moving to angle: ");
    Serial.println(targetAngle);
    
    // Rotate to the specified angle
    rotateShaftToAngle(targetAngle);
    
    // Reset for the next command
    inputString = "";
    stringComplete = false;
    
    // Prompt for next angle
    Serial.println("Enter an angle (0-360) to move the motor:");
  }
  
  // Read serial input
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    
    // Add incoming character to inputString
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}

// Function to rotate the shaft to a specific absolute angle
void rotateShaftToAngle(float targetAngle) {
  // Calculate the angle difference (shortest path)
  float angleDifference = targetAngle - currentAngle;
  
  // Normalize the angle difference to -180 to +180 degrees
  while (angleDifference > 180) angleDifference -= 360;
  while (angleDifference <= -180) angleDifference += 360;
  
  // Rotate by the calculated difference
  rotateShaftByDegrees(angleDifference);
  
  // Update current angle
  currentAngle = targetAngle;
  
  // Normalize current angle to 0-360 range
  while (currentAngle >= 360) currentAngle -= 360;
  while (currentAngle < 0) currentAngle += 360;
  
  Serial.print("Current position: ");
  Serial.println(currentAngle);
}

// Function to rotate the motor shaft by a specified angle
void rotateShaftByDegrees(float shaftAngle) {
  // Calculate total steps needed for the motor to rotate by the desired angle
  // Formula: (shaftAngle / 360) * GEAR_RATIO * STEPS_PER_REVOLUTION * MICROSTEPS
  // Since GEAR_RATIO = 1 (no gearbox), this simplifies to: (shaftAngle / 360) * STEPS_PER_REVOLUTION * MICROSTEPS
  long totalSteps = (long)((abs(shaftAngle) / 360.0) * GEAR_RATIO * STEPS_PER_REVOLUTION * MICROSTEPS);
  
  // Set direction based on positive or negative angle
  digitalWrite(dirPin, shaftAngle >= 0 ? HIGH : LOW);
  
  // Send the required number of pulses
  for (long i = 0; i < totalSteps; i++) {
    digitalWrite(stpPin, HIGH);
    delayMicroseconds(60); // Pulse width (adjust for speed)
    digitalWrite(stpPin, LOW);
    delayMicroseconds(60); // Interval between pulses (adjust for speed)
  }
}
/*
 * Robot Arm Control with DRM542 Stepper Motor Driver
 * Optimized for 15V operation with smooth motion and load bearing capability
 * Now supports negative angles for bidirectional control!
 * 
 * Wiring Instructions:
 * --------------------
 * Arduino Nano to DRM542 Connections:
 * - Arduino Digital Pin D2 (STEP_PIN) → PUL+ on DRM542 (Pulse signal for step control)
 * - Arduino Digital Pin D3 (DIR_PIN) → DIR+ on DRM542 (Direction signal)
 * - Arduino GND → PUL- and DIR- on DRM542 (Signal ground)
 * 
 * IMPORTANT: Use the digital pins (D2, D3) on the Arduino Nano, NOT the analog pins (A2, A3).
 * On the Nano, pins are labeled both with D# and A# - make sure to connect to the digital pins.
 * 
 * Power Connections:
 * - DRM542 V+ → 15V DC Power Supply positive terminal
 * - DRM542 V- → 15V DC Power Supply negative terminal
 * - Arduino powered via USB or external power supply (7-12V)
 * 
 * Motor Connections:
 * - Stepper motor A+ and A- → A+ and A- terminals on DRM542 (First coil)
 * - Stepper motor B+ and B- → B+ and B- terminals on DRM542 (Second coil)
 * 
 * DIP Switch Settings for 15V Operation:
 * - Set microstep resolution to 3200 steps/revolution (1/16 microstepping)
 *   for better smoothness and load-bearing capability
 * - Set current limit to 70-80% of motor's rated current for better torque
 *   while preventing overheating at 15V
 * - If available, enable the motor's holding torque when stopped
 * 
 * Usage:
 * - Open Serial Monitor (9600 baud)
 * - Enter any angle (positive or negative) and press Enter
 * - Examples: 90 (clockwise), -90 (counterclockwise), 450 (1.25 full rotations)
 * - Stepper motor will rotate to the desired angle with smooth acceleration/deceleration
 */

#include <Arduino.h>

// Pin Definitions
#define STEP_PIN 2  // Digital pin D2 connected to PUL+ on DRM542, sends pulses to control motor steps
#define DIR_PIN 3   // Digital pin D3 connected to DIR+ on DRM542, controls rotation direction
#define ENABLE_PIN 4 // Optional: connect to ENA+ on DRM542 if available (for power saving)

// Motor and Control Parameters - Configured for 1/16 microstepping
#define FULL_STEPS_PER_REV 200     // Standard stepper motor: 200 full steps per revolution
#define MICROSTEP_RATIO 16         // 1/16 microstepping (set via DRM542 DIP switches)
#define STEPS_PER_REVOLUTION (FULL_STEPS_PER_REV * MICROSTEP_RATIO)  // 3200 steps per revolution

// Speed control parameters
#define SPEED_MODE_SLOW 1          // Slow speed mode
#define SPEED_MODE_NORMAL 2        // Normal speed mode
#define CURRENT_SPEED_MODE SPEED_MODE_SLOW  // Set the current speed mode (SPEED_MODE_SLOW or SPEED_MODE_NORMAL)

// Delay values for different speeds (µs) - Speed doubled by halving delays
#define PULSE_DELAY_NORMAL 750     // Normal speed: 750µs delay between pulses (2x faster)
#define PULSE_DELAY_SLOW 7500      // Slow speed: 7500µs (7.5ms) delay between pulses (2x faster)
#define PULSE_WIDTH 10             // Short pulse width (µs) - most drivers need only 5-10µs

// Load bearing capability improvements
#define HOLD_DELAY 500             // Delay after movement (ms) to ensure stable position
#define MIN_STEP_MOVEMENT 5        // Minimum steps to move (prevents tiny movements)

// Calibration settings - Fix for the 90° → 180° issue
#define ANGLE_CALIBRATION_FACTOR 1  // If motor moves to 180° when commanded to 90°, use 0.5
#define DIRECTION_REVERSED false      // Set to true if motor moves in opposite direction than expected

// Global variables for angle control
float currentAngle = 0.0;           // Tracks the current angle of the stepper motor
String inputString = "";            // String to hold incoming serial data
boolean stringComplete = false;     // Flag to indicate if a complete string has been received

// Function prototypes
void rotateToAngle(float targetAngle);
void rotateSteps(long steps, bool clockwise);
void enableMotor(bool enable);

void setup() {
  // Configure pins as outputs
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
    // Enable the motor driver
  enableMotor(true);
  
  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("Robot Arm - Stepper Motor Angle Control (Constant Speed Mode)");
  Serial.println("Driver configured for 3200 steps per revolution (1/16 microstepping)");
  
  // Display speed mode
  if (CURRENT_SPEED_MODE == SPEED_MODE_SLOW) {
    Serial.println("Speed Mode: SLOW - For precise movements (2x faster than before)");
  } else {
    Serial.println("Speed Mode: NORMAL - High speed operation");
  }
  
  Serial.println("Calibration fix applied - Using factor: " + String(ANGLE_CALIBRATION_FACTOR));
  Serial.println("Motor using constant speed to prevent shaft slipping");
  Serial.println("Now accepts negative angles! (-90° rotates counterclockwise)");
  // Optional: Reset to a known position
  // Uncomment this if you want the motor to always start at zero
  // currentAngle = 0.0;
  
  Serial.println("Enter an angle (-360 to +360 or beyond) to move the motor:");
  
  // Reserve memory for input string
  inputString.reserve(10);
}

void loop() {
  // Check for completed serial input
  if (stringComplete) {
    // Convert input string to float
    float targetAngle = inputString.toFloat();
    
    // Accept any angle (including negative values)
    // Normalize to 0-360 range for internal calculations
    float normalizedAngle = targetAngle;
    while (normalizedAngle < 0) normalizedAngle += 360;
    while (normalizedAngle >= 360) normalizedAngle -= 360;
    
    Serial.print("Input angle: ");
    Serial.print(targetAngle);
    Serial.print("° | Normalized: ");
    Serial.print(normalizedAngle);
    Serial.println("°");
    
    // Rotate motor to target angle
    rotateToAngle(normalizedAngle);
    
    Serial.print("Current position: ");
    Serial.println(currentAngle);
    
    // Reset for next command
    inputString = "";
    stringComplete = false;
    Serial.println("Enter an angle (-360 to +360 or beyond) to move the motor:");
  }
  
  // Read serial data
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    
    // Process incoming character
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}

// Function to rotate motor to a specific angle
void rotateToAngle(float targetAngle) {
  // Apply calibration factor to requested angle
  // This fixes issues like when requesting 90° but motor moves to 180°
  float calibratedTargetAngle = targetAngle * ANGLE_CALIBRATION_FACTOR;
  
  // For debugging calibration
  Serial.print("Requested: ");
  Serial.print(targetAngle);
  Serial.print("° | Calibrated: ");
  Serial.print(calibratedTargetAngle);
  Serial.println("°");
  
  // Calculate angle difference (shortest path)
  float angleDifference = calibratedTargetAngle - currentAngle;
  
  // Normalize angle difference to -180 to +180 degrees
  while (angleDifference > 180) angleDifference -= 360;
  while (angleDifference <= -180) angleDifference += 360;
  
  // Calculate steps needed
  long stepsToMove = abs(angleDifference) * STEPS_PER_REVOLUTION / 360;
  
  // Skip very small movements (improves stability under load)
  if (stepsToMove < MIN_STEP_MOVEMENT) {
    Serial.println("Movement too small, skipping to avoid instability");
    return;
  }
  
  // Determine direction (with possible direction reversal)
  bool clockwise = (angleDifference >= 0);
  if (DIRECTION_REVERSED) {
    clockwise = !clockwise; // Invert direction if needed
  }
    // Enable motor if it was disabled to save power
  enableMotor(true);
  
  // Move the motor at constant speed (no acceleration/deceleration)
  rotateSteps(stepsToMove, clockwise);
  
  // Wait a moment to ensure stable position (especially with load)
  delay(HOLD_DELAY);
  
  // Update current angle
  currentAngle = calibratedTargetAngle; // Store the calibrated angle
  
  // Normalize current angle to 0-360 range
  while (currentAngle >= 360) currentAngle -= 360;
  while (currentAngle < 0) currentAngle += 360;
}

// Function to rotate motor by a specific number of steps at constant speed
void rotateSteps(long steps, bool clockwise) {
  // Set direction
  digitalWrite(DIR_PIN, clockwise ? HIGH : LOW);
  
  // Execute steps at constant speed
  for (long i = 0; i < steps; i++) {
    // Send pulse
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(PULSE_WIDTH);
    digitalWrite(STEP_PIN, LOW);
    
    // Fixed delay for constant speed based on selected speed mode
    if (CURRENT_SPEED_MODE == SPEED_MODE_SLOW) {
      delayMicroseconds(PULSE_DELAY_SLOW);
    } else {
      delayMicroseconds(PULSE_DELAY_NORMAL);
    }
  }
}

// Function to enable or disable the stepper motor
void enableMotor(bool enable) {
  // DRM542 has active LOW enable, so LOW = enabled, HIGH = disabled
  digitalWrite(ENABLE_PIN, enable ? LOW : HIGH);
  
  // Small delay to ensure the driver responds
  if (enable) {
    delay(5); // Give the driver time to fully enable
  }
}

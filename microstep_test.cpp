/*
 * Microstepping Test for DRM542 Stepper Motor Driver
 * This test helps verify if your microstepping configuration is working correctly
 * 
 * Instructions:
 * 1. Upload this test code to your Arduino
 * 2. Open Serial Monitor at 9600 baud
 * 3. Follow the test procedures to verify microstepping
 * 
 * Expected Results for Different Microstep Settings:
 * - Full Step (1:1): 200 steps = 360°, 50 steps = 90°
 * - Half Step (1:2): 400 steps = 360°, 100 steps = 90°
 * - 1/4 Step: 800 steps = 360°, 200 steps = 90°
 * - 1/8 Step: 1600 steps = 360°, 400 steps = 90°
 * - 1/16 Step: 3200 steps = 360°, 800 steps = 90°
 * - 1/32 Step: 6400 steps = 360°, 1600 steps = 90°
 */

#include <Arduino.h>

// Pin Definitions
#define STEP_PIN 2
#define DIR_PIN 3
#define ENABLE_PIN 4

// Test configurations - change these based on your DIP switch settings
#define FULL_STEPS_PER_REV 200     // Standard stepper motor: 200 full steps per revolution

// Common microstepping ratios
#define MICROSTEP_1_1    1         // Full step
#define MICROSTEP_1_2    2         // Half step  
#define MICROSTEP_1_4    4         // Quarter step
#define MICROSTEP_1_8    8         // 1/8 step
#define MICROSTEP_1_16   16        // 1/16 step
#define MICROSTEP_1_32   32        // 1/32 step

// Set this to match your DRM542 DIP switch setting
#define CURRENT_MICROSTEP MICROSTEP_1_2  // Change this to test different settings

// Calculate total steps per revolution based on microstepping
#define STEPS_PER_REVOLUTION (FULL_STEPS_PER_REV * CURRENT_MICROSTEP)

// Speed settings
#define PULSE_WIDTH 10
#define PULSE_DELAY 2000  // 2ms between pulses for clear observation

void setup() {
  // Configure pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  
  // Enable motor
  digitalWrite(ENABLE_PIN, LOW);  // DRM542 uses active LOW enable
  delay(10);
  
  // Initialize serial
  Serial.begin(9600);
  Serial.println("=== DRM542 Microstepping Test ===");
  Serial.println();
  Serial.print("Current microstepping setting: 1/");
  Serial.println(CURRENT_MICROSTEP);
  Serial.print("Steps per revolution: ");
  Serial.println(STEPS_PER_REVOLUTION);
  Serial.print("Degrees per step: ");
  Serial.println(360.0 / STEPS_PER_REVOLUTION, 4);
  Serial.println();
  
  Serial.println("Test Menu:");
  Serial.println("1 - Test 90° rotation (1/4 turn)");
  Serial.println("2 - Test 180° rotation (1/2 turn)");
  Serial.println("3 - Test 360° rotation (full turn)");
  Serial.println("4 - Test microstep resolution (1° movements)");
  Serial.println("5 - Continuous rotation test");
  Serial.println("r - Reverse direction");
  Serial.println();
  Serial.println("Enter test number:");
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();
    
    switch (command) {
      case '1':
        testAngle(90);
        break;
      case '2':
        testAngle(180);
        break;
      case '3':
        testAngle(360);
        break;
      case '4':
        testMicrostepResolution();
        break;
      case '5':
        continuousRotationTest();
        break;
      case 'r':
        testReverseDirection();
        break;
      default:
        Serial.println("Invalid command. Enter 1-5 or 'r'");
        break;
    }
    
    Serial.println("Enter test number:");
  }
}

void testAngle(float angle) {
  Serial.print("Testing ");
  Serial.print(angle);
  Serial.println("° rotation...");
  
  long steps = (angle / 360.0) * STEPS_PER_REVOLUTION;
  Serial.print("Calculated steps needed: ");
  Serial.println(steps);
  
  Serial.println("Motor should rotate clockwise. Mark starting position.");
  delay(2000);
  
  rotateSteps(steps, true);  // Clockwise
  
  Serial.print("Rotation complete. Motor should have moved exactly ");
  Serial.print(angle);
  Serial.println("°");
  Serial.println("Verify the actual movement matches expected angle.");
  Serial.println();
}

void testMicrostepResolution() {
  Serial.println("Testing microstep resolution with 1° movements...");
  Serial.println("Motor will make 10 steps of 1° each (total 10°)");
  Serial.println("With fine microstepping, these should be very smooth, small movements.");
  delay(2000);
  
  long stepsPerDegree = STEPS_PER_REVOLUTION / 360;
  Serial.print("Steps per degree: ");
  Serial.println(stepsPerDegree);
  
  for (int i = 1; i <= 10; i++) {
    Serial.print("Step ");
    Serial.print(i);
    Serial.print(" - Moving 1°...");
    
    rotateSteps(stepsPerDegree, true);
    delay(1000);
    Serial.println(" Done");
  }
  
  Serial.println("Microstep resolution test complete.");
  Serial.println("Movement should have been very smooth if microstepping is working.");
  Serial.println();
}

void continuousRotationTest() {
  Serial.println("Continuous rotation test - 5 full rotations");
  Serial.println("This tests for any cumulative errors in microstepping");
  delay(2000);
  
  for (int rotation = 1; rotation <= 5; rotation++) {
    Serial.print("Rotation ");
    Serial.print(rotation);
    Serial.println("/5");
    
    rotateSteps(STEPS_PER_REVOLUTION, true);
    delay(500);
  }
  
  Serial.println("Continuous rotation test complete.");
  Serial.println("Motor should be back at original position (within microstepping accuracy)");
  Serial.println();
}

void testReverseDirection() {
  Serial.println("Testing reverse direction - 90° counterclockwise");
  delay(2000);
  
  long steps = STEPS_PER_REVOLUTION / 4;  // 90 degrees
  rotateSteps(steps, false);  // Counterclockwise
  
  Serial.println("Reverse rotation test complete.");
  Serial.println();
}

void rotateSteps(long steps, bool clockwise) {
  digitalWrite(DIR_PIN, clockwise ? HIGH : LOW);
  delay(1);  // Allow direction to settle
  
  for (long i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(PULSE_WIDTH);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(PULSE_DELAY);
  }
}

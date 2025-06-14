// DRM542 servo motor: The project is designed to control a DRM542 servo motor.

#include <Arduino.h>
#define STEP_PIN 2
#define DIR_PIN 3

void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, HIGH); // Set rotation direction
}

void loop() {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(1250);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(1250);
}

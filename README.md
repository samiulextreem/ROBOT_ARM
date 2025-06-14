# ROBOT ARM Project

## Overview
This project controls a DRM542 servo motor for a robot arm using an Arduino Nano board. The system is designed to provide precise motor control for robotic arm applications.

## Hardware Components
- Arduino Nano (ATmega328)
- DRM542 Servo Motor Driver
- Stepper motor

## Pin Configuration
- STEP_PIN: 2 (Pulse signal to control motor steps)
- DIR_PIN: 3 (Direction control pin)

## Features
- Simple stepper motor control
- Direction control
- Microsecond timing for precise movement

## Getting Started
1. Clone this repository: `git clone https://github.com/samiulextreem/ROBOT_ARM.git`
2. Open the project in PlatformIO
3. Connect your Arduino Nano to the DRM542 driver according to the pin configuration
4. Upload the code to your Arduino

## Project Structure
- `src/main.cpp`: Main source code
- `platformio.ini`: PlatformIO configuration

## Future Improvements
- Add acceleration/deceleration profiles
- Implement position control
- Add multi-axis support for full arm movement

## License
This project is open source and available for anyone to use and modify.

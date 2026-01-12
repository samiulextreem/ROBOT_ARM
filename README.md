#  2-DOF Robot Arm Controller

A complete control system for a 2-degree-of-freedom (2-DOF) robot arm, featuring Arduino-based firmware for low-level motor control and Python-based software for inverse kinematics and path planning.

##  Overview

This project provides both the firmware and the control software for a custom-built robot arm.
- **Firmware**: Runs on Arduino Nano using PlatformIO. Controls stepper motors via DRM542 drivers.
- **Software**: Python scripts for serial communication, calibration, and inverse kinematics control.

##  Demonstration

Check out the [demonstration video](demonstration_video.mp4) to see the arm in action.

##  Hardware Components

- **Controller**: Arduino Nano (ATmega328)
- **Drivers**: DRM542 Stepper Motor Drivers
- **Actuators**: Stepper Motors (NEMA 17/23 compatible)
- **Power Supply**: Appropriate voltage for your motors/drivers

### Pin Configuration (Arduino Nano)

| Function | Pin | Description |
|----------|-----|-------------|
| ENABLE   | D5  | Motor enable signal |
| STEP     | D6  | Pulse signal for steps |
| DIR      | D7  | Direction control |

##  Getting Started

### 1. Firmware Setup (PlatformIO)

The firmware is located in the `src/` folder and is managed by PlatformIO.

1.  Open the project folder in VS Code with the PlatformIO extension installed.
2.  Connect your Arduino Nano via USB.
3.  Build and Upload the firmware:
    `ash
    pio run --target upload
    ``n
### 2. Python Environment Setup

The control software requires Python 3 and a few dependencies.

1.  Install Python dependencies:
    `ash
    pip install pyserial matplotlib numpy
    ``n
### 3. Connection

1.  Identify the COM ports for your Arduino boards.
2.  Update `robot_controller_calibration.py` and `serial_sender.py` with your specific COM port numbers:
    `python
    PORT1 = 'COM11'   # Update this
    PORT2 = 'COM10'   # Update this
    ``n
##  Usage

### Calibration & Control
Run the controller script to calculate kinematics and move the arm:
`ash
python robot_controller_calibration.py
``nThis script handles:
-   Inverse kinematics calculation (Target (x,y) -> Angles)
-   Visualizing the arm position
-   Sending commands to the firmware

### Direct Serial Testing
Use the sender script to test raw connections:
`ash
python serial_sender.py
``n
##  Project Structure

- `src/main.cpp`: C++ firmware for stepper control.
- `robot_controller_calibration.py`: Main Python control script with Inverse Kinematics.
- `serial_sender.py`: Utility for serial port testing.
- `platformio.ini`: PlatformIO build configuration.
- `demonstration_video.mp4`: Video showcase.

##  Future Improvements

- [ ] Add acceleration/deceleration profiles to firmware.
- [ ] Implement G-code interpreter.
- [ ] Add GUI for easier control.
- [ ] 3D simulation of the arm state.

##  License

This project is open source. Feel free to use and modify!

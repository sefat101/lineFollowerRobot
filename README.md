# Line Following Robot with PID Control

This repository contains the Arduino code and documentation for a line-following robot with PID control. The robot uses infrared (IR) sensors to follow a black line on a white surface.

## Overview

The project involves an Arduino-based line-following robot with two infrared sensors and an L298N motor driver. The code implements a PID controller to provide more precise control over the robot's movements.

## Features

- PID control for smooth line following
- Adjustable PID constants for tuning performance
- Real-time sensor readings and PID output monitoring through serial communication

## Hardware Requirements

- Arduino board (e.g., Arduino Uno)
- Infrared (IR) sensors (2)
- L298N motor driver
- DC motors (2)
- Chassis and wheels
- Power supply
- Jumper wires

## Software Requirements

- Arduino IDE (https://www.arduino.cc/en/software)

## Getting Started

1. Connect the hardware components according to the wiring diagram.
2. Open the Arduino IDE and upload the provided code to your Arduino board.
3. Monitor the robot's behavior through the Serial Monitor for debugging and tuning.

## Wiring Diagram

Include a simple wiring diagram showing the connections between the Arduino, IR sensors, motor driver, and motors.

## Code Explanation

Briefly explain the key sections of the code, such as sensor reading, PID calculation, and motor control.

## Tuning PID Constants

Describe how users can experiment with and tune the PID constants (`KP`, `KD`, `KI`) to achieve optimal line-following performance.

## Troubleshooting

Provide common troubleshooting tips for issues such as sensor misalignment, motor problems, or unexpected behavior.

## Contributing

If you'd like to contribute to the project, fork the repository and submit a pull request. Feel free to suggest improvements, report issues, or add new features.





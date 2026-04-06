Stewart Platform IK Simulator & Serial Control

This project implements the inverse kinematics (IK) of a 6-DOF Stewart platform, including real-time visualization and serial communication with an ESP32 for controlling servos via a PCA9685 driver.

🧩 Overview

The system is divided into two main parts:

Simulation (Python)
Inverse kinematics calculation
Visualization of simplified platform position
Hardware Control (ESP32 + PCA9685)
Receives servo commands via serial communication
Controls 6 servos (MG996) using PWM driver
⚙️ Hardware Setup
Microcontroller: ESP32
Servo Driver: PCA9685 (I2C)
Servos: MG996 (or similar high-torque servos)

The PCA9685 allows precise PWM control of multiple servos using I2C, reducing load on the ESP32.

🔌 System Architecture

Python (IK + Control)
→ Serial Communication (USB)
→ ESP32
→ I2C
→ PCA9685
→ Servos
→ Stewart Platform

🧠 Inverse Kinematics (Python)
📌 ik_visualizer.py

This script simulates the Stewart platform and visualizes its motion.

The user inputs:
Roll
Pitch
The system computes:
The required position of each actuator (6 servos)
Output:
Individual servo values corresponding to platform orientation
Graphical visualization of position

📌 ik_serial.py

This script connects directly to the ESP32 via serial communication.

🔁 Workflow:
User inputs:
Roll
Pitch
Python:
Computes inverse kinematics
Normalizes the output values
Sends data via serial in the format:
a,b,c,d,e,f#

Where:

a–f are integer values (one per servo)
# indicates end of message
Waits for next user input

🔧 ESP32 Firmware (Arduino Framework)
📌 controlSerial.ino

This code runs on the ESP32 and handles servo control.

🔁 Workflow:
Waits for a complete serial message ending in #
Parses the received string
Splits values into 6 independent variables
Updates each servo position
🎛️ Servo Control Logic

Each servo is controlled using:

servo.write(90 + m);

Where:

90 → neutral (center) position
m → normalized offset value from IK

👉 This means:

m = 0 → servo at center position
Positive/negative values → relative movement
📡 Serial Communication Format
a,b,c,d,e,f#

📦 Requirements (Python)
pip install numpy matplotlib pyserial

📸 Model & References
Images and screenshots available in /images
3D model available on GrabCAD https://grabcad.com/library/stewart-plataform-1
📝 Notes
This repository focuses on inverse kinematics and low-level control
Advanced control strategies (PID, LQR, BC+DeepRL) are implemented in a separate project
Designed for modular integration with computer vision systems

👨‍💻 Author
Cristian Guerrero

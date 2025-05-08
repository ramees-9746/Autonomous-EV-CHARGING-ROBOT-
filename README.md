# Autonomous EV Charging Robot

This project showcases an autonomous electric vehicle (EV) charging robot that detects the charging port using computer vision and connects the charger without manual intervention. It is designed to assist EV owners by automating the charging process using a robotic arm and object detection.

## Key Features
- Object detection using **OpenCV** and **Raspberry Pi camera**
- Robotic arm control with **servo motors**
- Line-following navigation using **IR sensors**
- Distance alignment using **ultrasonic sensor**
- Controlled via **Python scripts** running on **Raspberry Pi 4B**

## Components Used
- Raspberry Pi 4B
- Pi Camera
- 3x Servo Motors (for robotic arm)
- 2x DC Motors (for navigation)
- Ultrasonic Sensor (HC-SR04)
- IR Sensors (for line following)
- Motor Driver (L298N or similar)
- Battery Pack
- Chassis and Wheels

## Project Phases
1. **Navigation** – Robot follows a black line to reach the charging station.
2. **Port Detection** – Uses camera + OpenCV to detect the EV's charging port.
3. **Alignment** – Calculates distance to the port using ultrasonic sensor and aligns itself.
4. **Charging** – Robotic arm moves and connects the charger to the detected port.

## Tools and Technologies
- **Python**
- **OpenCV**
- **GPIO Zero / RPi.GPIO**
- **Embedded electronics (motors, sensors)**

## How It Works
1. The robot follows a predefined path using line sensors.
2. Once it detects the EV, it switches to object detection mode.
3. It aligns itself using distance measurements.
4. The robotic arm positions the plug into the charging port.

## Future Improvements
- Implement wireless communication for remote control
- Add feedback from the charging system (voltage/current)
- Improve object detection with machine learning

## Demo
https://drive.google.com/file/d/1pVLsI9helDQLATfJ_irLulIkD04gWD7N/view?usp=drivesdk

## Author
Ramees Muhammad K  
B.Tech in Electrical and Electronics Engineering

## Team Contribution
This was a team project completed as part of our B.Tech final year work.

### My Role:
- Developed the object detection system using OpenCV and Python
- Integrated the Raspberry Pi camera with the control system
- Programmed servo motor control for the robotic arm
- Worked on the logic for distance-based alignment using the ultrasonic sensor

Other team members contributed to:
- Mechanical design and chassis setup
- Line-following motor control
- Circuit assembly and hardware testing

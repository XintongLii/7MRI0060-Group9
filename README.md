# Medical Robotics Project (7MRI0060 - Group 9)

This repository contains all the resources, designs, and code for a SCARA-based medical robotic arm developed as part of the **7MRI0060 Applied Medical Robotics** module. The project demonstrates a robotic system capable of precise movement and enhanced adaptability in healthcare applications.

## Overview

The SCARA robotic arm is designed to perform rotational and translational movements using modular components. The project includes three generations of design improvements, focusing on structural integrity, adaptability, and operational efficiency. It integrates advanced control systems, including forward and inverse kinematics, and features real-time feedback via a GUI.

---

## Repository Structure

- **`/CAD`**: Contains 3D models and CAD designs for the robotic arm.
- **`/Arduino`**: Arduino source code for low-level motor control.
- **`/MATLAB`**: MATLAB code for high-level control, including kinematics simulation and GUI.

---

## Project Components

### 1. CAD/3DP Design

- **Description**:
  - Initial designs focused on fixed gripping mechanisms.
  - Improved arm strength and adaptability through modular redesigns.
  - Current design includes a separated storage box for enhanced strength and reduced load.

- **Features**:
  - Modular assembly for ease of maintenance.
  - Adjustable gripping mechanism to accommodate various pen types.
  - CAD files and 3D printing parameters are available in the `/CAD` folder.

---

### 2. Hardware Integration

- **Description**:
  - The robotic system uses 2 motors, an H-bridge board, Arduino, and other hardware for movement control.

- **Hardware Components**:
  - **Motors**: Drive the robotic arms for rotational and linear motion.
  - **Arduino**: Processes low-level commands for motor control.
  - **H-Bridge**: Ensures smooth direction switching and speed control.

- **Files**:
  - Hardware assembly instructions and component details can be found in the `/Arduino` directory.

---

### 3. Low-level Control (Arduino)

- **Description**:
  - Implements PID control to fine-tune motor movement.
  - Parameters for optimal performance were determined through iterative testing.

- **PID Results**:
  - Motor 1 (M1): Kp = 17.5, Ki = 8, Kd = 10
  - Motor 2 (M2): Kp = 12, Ki = 7, Kd = 5
  - Errors:  
    - M1: 40.89° ± 0.23°  
    - M2: 2.76° ± 0.17°

- **Usage**:
  - Upload the Arduino scripts from the `/Arduino` directory to your board to control the robot.

---

### 4. High-level Control (MATLAB)

- **Description**:
  - Controls the robotic arm via a MATLAB GUI.
  - Implements forward kinematics (FK) and inverse kinematics (IK) for position control.
  - 
- **Usage**:
  - Run the MATLAB scripts in `/MATLAB` to simulate and control the robot.

---

## Limitations & Future Work

### Limitations
- The system’s precision is limited due to:
  - Motor resolution.
  - Structural flex in the arms.
  - Variability in PID performance.

### Future Improvements
1. Add sensors for precise angle measurement.
2. Introduce a gear system to reduce motor speed and improve accuracy.
3. Replace current motors with stepper motors for finer control.
4. Refine simulation models to better reflect real-world conditions.

---

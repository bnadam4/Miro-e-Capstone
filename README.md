# Miro-e-Capstone

Welcome to the **MiRo-E** Social Robot Project repository! This ongoing capstone project focuses on the development of **MiRo-E**, a social robot designed to support pediatric patients in healthcare settings. Our goal is to create a therapeutic and interactive robot that helps alleviate pediatric anxiety through non-pharmacological interventions. This project is being developed, with key features currently being implemented and refined.

## Project Overview

**MiRo-E** is an interactive, comforting robot designed to serve as a companion for children in hospitals. It aims to provide therapeutic support to young patients by engaging them in activities such as guided breathing exercises, interactive games, and other relaxation techniques. The robot is also equipped with the ability to recognize QR/ARUCO codes, which trigger various activities, making it a versatile tool for both patients and healthcare professionals.

### Key Features Under Development

The development of **MiRo-E** is focused on the following core features:

- **Guided Breathing Exercises:** Helping children manage anxiety through calming breathing techniques.
- **Interactive Play Modes:** Engaging children with games and activities to distract and entertain them during their hospital stay.
- **QR/ARUCO Code Recognition:** Enabling easy activity initiation by scanning QR or ARUCO codes.
- **Sensor Integration:** Detecting motion and touch to tailor interactions based on the child’s needs.

## Development Setup

The **MiRo-E** project is built using the following technology stack:

- **Operating System:** Ubuntu
- **Robot Operating System (ROS):** Utilized for robot control, sensor management, and communication.
- **Programming Language:** Python, with libraries such as OpenCV, and ROS Py.

## Project Structure

The project consists of two main processes:

1. **Supervisor Process:** This process allows a supervisor (e.g., healthcare staff) to power on the robot, activate various modes and activities, and monitor the robot’s progress. The supervisor can manage transitions between different modes through an app or software interface.
2. **End-User Process:** This process relates to the interaction between the robot and the child. Children will engage with the robot through touch sensors, QR/ARUCO code scanning, or voice commands. The robot will then initiate a pre-programmed activity and interact with the child until the activity concludes.

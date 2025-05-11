# MiRo-e Capstone: C.A.R.E. Project  
### *Child Anxiety Relief Experience*

Welcome to the **MiRo-E Social Robot Project** repository! This capstone project explores the therapeutic potential of **MiRo-E**, a commercially available robot developed by [Consequential Robotics](https://consequentialrobotics.com/miroe), in pediatric healthcare environments.

Our goal is to design an **interactive, non-pharmacological tool** to help alleviate anxiety in young patients using personalized, engaging activities.


## 🧠 Project Overview

The **C.A.R.E. Project** is focused on creating a supportive experience for pediatric patients by using MiRo-E to guide them through:

- **Guided Breathing Exercises**
- **Muscle Relaxation Activities**
- **Audiobook Storytelling**
- **Dance & Movement Games**

These therapeutic activities are designed to reduce anxiety and can be triggered through multiple input methods:
- **QR/ARUCO Codes**
- **Voice Commands**
- **Remote Control Interface**

## 🤖 MiRo-E Features

MiRo-E is equipped with multiple sensors that allow it to perceive and respond to its environment. Key features include:

### 🎯 Interactive LED Color Modes

- **Blue** – Idle exploration mode  
  - MiRo looks around curiously  
  - Responds to touch on head/back  
  - Ears and tail can be gently crinkled  

- **Green** – Face detection active

- **Yellow** – Listening mode (triggered after hearing “MiRo” for 10 seconds)

### 🎭 Activity Modes
![image](https://github.com/user-attachments/assets/d479e009-7a52-4aaf-bdc8-ef97deb19ea5)


| Activity               | Description                                                                 |
|------------------------|-----------------------------------------------------------------------------|
| **Breathing Exercises**| Guides children through calming breathing routines.                         |
| **Muscle Relaxation**  | Leads light stretching to ease physical tension.                            |
| **Audiobooks**         | Narrates a story while responding with motions and animations.              |
| **Dance**              | Encourages play and movement to uplift mood.                                |


## 🕹️ Activity Triggers

MiRo-E can be activated in several ways for maximum accessibility:
![image](https://github.com/user-attachments/assets/98b78a6c-60ae-49a1-9965-d9ff67d30ce7)


- **QR/ARUCO Code Recognition** – Show a corresponding code to MiRo's camera.
- **Voice Command** – Say “**MiRo**” followed by the activity name.
- **Remote Control** – Tap the corresponding icon on the screen.


## 🛠️ Development Setup

### 🔧 Robot System

- **Operating System**: Ubuntu
- **Robot Operating System (ROS)**: Manages control, sensors, and messaging
- **Programming Language**: Python  
  - Microsoft Azure API  
  - OpenCV  
  - Pillow  
  - ROS Py

### 🖲️ Remote Control Device

- **Components**: ESP32 microcontroller, Bluetooth, 5” touchscreen
- **Programming Language**: C
- **Framework**: ESP-IDF


## 📷 Hardware Overview

![image](https://github.com/user-attachments/assets/11bc610d-2be3-4ad5-91e1-8f9788c3b3ea)


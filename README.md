# Project EVA — Humanoid Robot

**EVA (Enhanced Visitor Assistant)** is a humanoid robot designed to function as an
autonomous visitor guide and professor assistant. Built as a B.Tech major project by
a team of 5, I was responsible for all embedded systems integration and ROS2 software
— from the Micro-ROS motor firmware to SLAM navigation and MoveIt2 arm control.

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-Apache_2.0-green)](LICENSE.md)

---

## Demo

> 📸 RViz + Gazebo screenshots, real world testing videos -> https://tinyurl.com/Project-EVA

| Full Robot (RViz) | Gazebo Simulation | Physical Build |
|---|---|---|
| ![EVA RViz](Images/Namaste2.png) | ![EVA Gazebo](Images/Gazebo1.png) | ![EVA Physical](Images/IMG_8677.JPG) |

---

## What I Built

This wasn't a tutorial follow-along — here's what was actually custom-developed:

- **Custom drive kinematics node** — EVA uses a non-standard wheel configuration.
  I wrote a ROS2 node (`project_eva_drive`) that translates `cmd_vel` messages into
  per-wheel motor commands for this specific geometry, something Nav2 doesn't handle
  out of the box.

- **Micro-ROS embedded firmware** — The robotic arms are controlled by a
  microcontroller running Micro-ROS. This bridges the ROS2 graph directly to the arm motor drivers over
  serial, enabling real-time joint control from MoveIt2.

- **MoveIt2 arm configuration** — Configured MoveIt2 for EVA's custom arm URDF,
  including SRDF, kinematics solver setup, and motion planning for both arms with
  finger actuation.

- **Full URDF from scratch** — Modelled the entire robot in URDF/Xacro: body,
  dual arms, finger joints, and the custom drive base. Verified in RViz and simulated
  in Gazebo with ros2_control.

- **Autonomous navigation** — Integrated Nav2 with SLAM Toolbox for indoor
  navigation. Used RPlidar, camera, and IMU fusion for localization.

- **Voice interaction system** — Built an NLP-based command interface allowing
  visitors to interact with EVA using natural language.

---

## System Architecture

```
Voice Input ──► NLP Node ──► Task Planner
                                  │
                    ┌─────────────┼──────────────┐
                    ▼             ▼               ▼
              Nav2 Stack     MoveIt2 Arms     TTS Response
                    │             │
                    ▼             ▼
             Drive Node    Micro-ROS Bridge
                    │             │
             Motor Drivers   Arm Motor Drivers
```

---

## Hardware

| Component | Details |
|---|---|
| Compute | Raspberry Pi & Jetson (ROS2) + Teensy (Micro-ROS) |
| LiDAR | RPLidar A2 |
| Camera | Intel RealSense |
| IMU | BNO055 |
| Drive | Custom omni base|
| Arms | servo & DC motor|
| Body | Custom 3D printed + machined frame |

---

## ROS2 Package Structure

```
project_eva/          — URDF, launch files, robot description
project_eva_drive/    — Custom drive kinematics controller
project_eva_controller/ — Micro-ROS bridge for arm control
project_eva_moveit/   — MoveIt2 config (SRDF, kinematics, planning)
```

---

## Getting Started

### Prerequisites
- ROS2 Humble
- MoveIt2
- Nav2
- SLAM Toolbox
- Micro-ROS (for embedded arm control)

### Build

```bash
mkdir -p ~/project_eva_ws/src && cd ~/project_eva_ws/src
git clone https://github.com/The-Kriz/project_eva
cd ..
colcon build --symlink-install
source install/setup.bash
```

### Run

**Simulation (Gazebo + RViz):**
```bash
ros2 launch project_eva launch_sim.launch.py use_sim_time:=true use_omni_wheel:=true use_drive:=true use_eva:=true
```

**Robot State Publisher only:**
```bash
ros2 launch project_eva rsp.launch.py use_sim_time:=true use_omni_wheel:=false use_drive:=true use_eva:=true
```

---

## Skills Demonstrated

`ROS2` `MoveIt2` `Nav2` `SLAM` `Micro-ROS` `URDF/Xacro` `Gazebo`
`Embedded C/C++` `Custom Kinematics` `Sensor Fusion` `NLP` `Python` `C++`

---

## Team

Built by a team of 5 at SRM Institute of Science and Technology (B.Tech Major Project, 2024).
My scope: all ROS2 software, embedded firmware, and hardware integration.

---

*Part of my robotics portfolio — see also [rog_ros_bot](https://github.com/The-Kriz/rog_ros_bot)*


## 1 Start
### 1.1 Download

    cd $HOME
    mkdir project/src
    cd project/src
    git clone https://github.com/The-Kriz/project_eva

### 1.2 Build

    colcon build --symlink-install
    source install/setup.bash
    
### 1.2 Run

#### 1.2.1 Run Robot State Publisher
    ros2 launch project_eva rsp.launch.py use_sim_time:=true use_omni_wheel:=false use_drive:=true use_eva:=true


#### 1.2.2 Run Robot Simulation
    ros2 launch project_eva launch_sim.launch.py use_sim_time:=true use_omni_wheel:=true use_drive:=true use_eva:=true

## 2 URDF

### 2.1 Drive
![Drive_RVIZ](https://github.com/The-Kriz/project_eva/assets/90817926/9f931c61-958a-44ea-8d56-4918f714136c)
![Drive_Gazebo](https://github.com/The-Kriz/project_eva/assets/90817926/77d0dbb7-fc24-44be-a96e-c7d7c1400215)

### 2.2 Arm
![Arm_RVIZ](https://github.com/The-Kriz/project_eva/assets/90817926/d6fc030a-5922-4023-88fe-fe54f42359e4)

### 2.3 Fingers
![Fingers_RVIZ](https://github.com/The-Kriz/project_eva/assets/90817926/1fa3b454-476e-470e-9692-c7e8a863c41d)

### 2.4 EVA
![EVA_RVIZ](https://github.com/The-Kriz/project_eva/assets/90817926/73d8d449-0a14-4c71-b97e-e2c0f381ddf0)


## 3 POSE
![Namaste_RVIZ](https://github.com/The-Kriz/project_eva/assets/90817926/a8ccb727-971d-4e16-8bf1-e1d1d1865f27)

![Namaste_Gazebo](https://github.com/The-Kriz/project_eva/assets/90817926/49b6eb77-ddec-40f3-b5f3-dc92ccfeb3f8)


![a](https://github.com/The-Kriz/project_eva/blob/main/Images/IMG_8634.JPG)
![b](https://github.com/The-Kriz/project_eva/blob/main/Images/IMG_8635.JPG)
![c](https://github.com/The-Kriz/project_eva/blob/main/Images/IMG_8677.JPG)
![d](https://github.com/The-Kriz/project_eva/blob/main/Images/IMG_8678.JPG)
![e](https://github.com/The-Kriz/project_eva/blob/main/Images/IMG_8720.JPG)
![f](https://github.com/The-Kriz/project_eva/blob/main/Images/IMG_8715.JPG)


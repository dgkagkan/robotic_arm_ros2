# 🤖 Robotic Arm ROS2 — Pick & Place with Computer Vision

A custom 6-DOF robotic arm with a dual-prismatic gripper, capable of autonomous pick-and-place operations using color-based object detection.

[![Demo Video](https://img.shields.io/badge/▶_Demo-YouTube-red?style=for-the-badge&logo=youtube)](https://youtu.be/iawmaIgp83U?si=qB1XLPUv3vmfE5jV)

---

## Overview

This project implements a complete pick-and-place pipeline in simulation:

1. **Vision Server** — Detects colored cubes (red, blue, green) via camera using OpenCV HSV filtering
2. **Action Server** — Orchestrates the pick-and-place sequence using MoveIt2 with a goal queue system
3. **Teleop Client** — Keyboard interface to send goals and cancel mid-motion

The robot identifies a target cube by color, plans a path to it, picks it up, and stacks it at a predefined location.

---

## Features

- 6-DOF custom arm with dual-prismatic gripper (workaround for mimic joint in Gazebo)
- Color-based object detection (HSV) with real-time debug image stream
- Goal queue — send multiple pick requests, executed sequentially
- Mid-motion cancel — press `C` to stop the robot immediately during any movement
- Cartesian path planning for approach/retreat
- Automatic cube stacking with incremental height

---

## Prerequisites

- **Ubuntu 24.04**
- **ROS2 Jazzy**
- **Gazebo Harmonic**
- **MoveIt2**
- **OpenCV**
- **cv_bridge**

---

## Installation

### 1. Install ROS2 Jazzy

Follow the [official ROS2 Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).

### 2. Install dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-moveit \
  ros-jazzy-ros-gz \
  ros-jazzy-cv-bridge \
  ros-jazzy-rqt-image-view \
  ros-jazzy-controller-manager \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-trajectory-controller
```

### 3. Clone and build

```bash
mkdir -p ~/robotic_arm_ws/src
cd ~/robotic_arm_ws/src
git clone https://github.com/dgkagkan/robotic_arm_ros2.git .
cd ~/robotic_arm_ws
colcon build
source install/setup.bash
```

---

## Usage

### Option 1: Launch everything at once

```bash
ros2 launch robotic_arm_bringup full_system.launch.py
```

This starts Gazebo, MoveIt2, the vision server, the action server, and the teleop client (in a separate terminal).

### Option 2: Launch step by step

**Terminal 1** — Gazebo simulation:
```bash
ros2 launch robotic_arm_bringup gazebo.py
```

**Terminal 2** — MoveIt2:
```bash
ros2 launch robotic_arm_bringup moveit.launch.xml
```

**Terminal 3** — Vision server:
```bash
ros2 run robotic_arm_commander optical_server
```

**Terminal 4** — Action server:
```bash
ros2 run robotic_arm_commander optical_action_server
```

**Terminal 5** — Teleop:
```bash
ros2 run robotic_arm_commander teleop
```

### Camera debug view (optional)

```bash
ros2 run rqt_image_view rqt_image_view /camera/debug_image
```

---

## Controls

| Key | Action |
|-----|--------|
| `R` | Pick red cube |
| `B` | Pick blue cube |
| `G` | Pick green cube |
| `C` | Cancel current goal (stops robot mid-motion) |
| `Q` | Quit teleop |

---

## Architecture

```
┌──────────┐    goal     ┌──────────────────┐   service   ┌───────────────┐
│  Teleop  │────────────▶│  Action Server   │────────────▶│ Vision Server │
│ (Client) │◀────────────│  (Pick & Place)  │◀────────────│   (OpenCV)    │
│          │   result    │                  │  cube pose  │               │
└──────────┘             │    MoveIt2       │             │  /camera/     │
                         │    arm_          │             │  image_raw    │
                         │    gripper_      │             └───────────────┘
                         └──────────────────┘
```

---

## Packages

| Package | Description |
|---------|-------------|
| `robotic_arm_description` | URDF/XACRO, meshes |
| `robotic_arm_moveit_config` | MoveIt2 configuration (SRDF, controllers) |
| `robotic_arm_bringup` | Launch files, world files, Gazebo config |
| `robotic_arm_commander` | Action server, vision server, teleop client |
| `robotic_arm_interfaces` | Custom action and service definitions |

---

## Custom Interfaces

**Action — Controller.action:**
```
# Goal
string color
---
# Result
float64 result_x
float64 result_y
string msg
---
# Feedback
float64 current_x
float64 current_y
```

**Service — PickTarget.srv:**
```
# Request
string target_color
---
# Response
float64 x
float64 y
float64 z
float64 roll
float64 pitch
float64 yaw
float64 grasp_width
bool success
```

---

## License

This project is open source.

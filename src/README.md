# Robotic Arm ROS2 Project

A ROS2-based robotic arm simulation using **Gazebo** for physics and **MoveIt2** for motion planning.

## Packages
- `robotic_arm_description`: URDF and Xacro files.
- `robotic_arm_moveit_config`: MoveIt2 configuration.
- `robotic_arm_commander`: C++ Action Server and motion control logic.
- `robotic_arm_bringup`: Launch files to start the simulation.
- `robotic_arm_interfaces`: Custom messages and actions.

## Requirements
- ROS2 (jazzy)
- MoveIt2
- Gazebo ROS Packages

## How to Run
1. Build the workspace:
   ```bash
   colcon build --symlink-install
   source install/setup.bash
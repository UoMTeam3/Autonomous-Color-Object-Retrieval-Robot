# Manipulator Subsystem

This directory contains the **mainpulator code** used by the Autonomous Color Object Retrieval Robot.
It enables the manipulator to **take coordinates, move the end effector to those coordinates, attempt to grasp any item at that area and then return whether it succesfully grasped or not**.

The manipulator uses a modified version of the **The PyMyCobot Library**.

---

# Table of Contents

1. Folder Structure
2. MyCobot280Pi
3. Installation
4. Running the Navigation System
5. Dependencies
6. Contributors

---

# Folder Structure

```
Manipulator/
|    MyCobot_ros2_team3/
|    │
|    ├── mycobot_280/
|    |      mycobot_280pi/
|    |          mycobot_280pi/ (They are named the same thing)
|    │
|    └── README.md
│
└── README.md
```

mycobot280pi is the only package that needs to be run for this.

---

# MyCobot280Pi

# Package Overview

| Package                               | Purpose                                          |
| ------------------------------------- | ------------------------------------------------ |
| `mycobot_280`                         | ROS2 & Python Interface for Manipulator          |

---

# mycobot_280

External Repository used: 
Repository cloned using:

  git clone https://github.com/elephantrobotics/mycobot_ros

## Purpose

This package provides the **description of the manipulator**, allowing the robot to **perform inverse kinematics** in order to set joint parameters.

### Important Files

```
mycobot_280/
│
├── mycobot_280pi/
│   ├──  mycobot_280pi/
│   |    └──  Custom_Node_Control.py
│   └──  launch/
└── 
```

### Key Functionality

* Takes position coordinates from two topics, then directs the manipulator to those coordinates. With different behaviors depending on which topic it was sent on, with it picking up the block and grasping if it sent on the topic:
```
/Do_Not_Read
```
And moving a block (assumedly grasped), and dropping it at the coordinates of the point it send on:
```
/Bin_Topic
```
### Modifications / Configuration

For this project:

* Added a custom python file to intergrate topics. Custon_Node_Control.py.

Example run (while SSH'd into the manipulator):

```
ros2 run mycobot_280pi Custom_Node_Control
```

---

# Installation

Clone the repository:

```
git clone https://github.com/UoMTeam3/Autonomous-Color-Object-Retrieval-Robot.git
```

Build the workspace:

```
colcon build
```

Source the workspace:

```
source install/setup.bash
```

# Running the Manipulator Node

### 1 Start the Node


ros2 run mycobot_280pi Custom_Node_Control


# Dependencies

The manipulator node depends on the following ROS2 packages:

```
actionlib
controller_manager
join_state_publisher
join_state_publisher_gui
joint_state_publisher
joint_state_publisher_gui
joy
message_generation
message_runtime
moveit_configs_utils
moveit_kinematics
moveit_planners
moveit_ros_move_group
moveit_ros_visualization
moveit_ros_warehouse
moveit_setup_assistant
moveit_simple_controller_manager
mycobot_description
python-tk
rclcpp
rclpy
robot_state_publisher
rosidl_default_generators
rosidl_default_runtime
rviz2
rviz_common
rviz_default_plugins
std_msgs
tf2_ros
warehouse_ros_mongo
xacro
```

Install dependencies using:

```
rosdep install --from-paths src --ignore-src -r -y
```

---

# Contributors

## Contributors

- [Benjamin Woodruff](https://github.com/Ben-Woodruff)

University of Manchester - Autonomous Robotics Project

---

# License

Each package contains its own license file.
Refer to the individual package directories for licensing details.

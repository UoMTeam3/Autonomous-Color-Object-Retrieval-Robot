# Navigation System

This directory contains the **complete navigation stack** used by the Autonomous Color Object Retrieval Robot.
It enables the robot to **map environments, localize itself, navigate autonomously, and explore unknown areas**.

The navigation system is built on **ROS2 Navigation Stack (Nav2)** and integrates:

* **RPLidar** for environment perception
* **SLAM Toolbox** for mapping
* **AMCL** for localization
* **Nav2** for path planning and control
* **Wavefront Frontier Detection** for autonomous exploration

---

# Table of Contents

1. Folder Structure
2. Navigation Architecture
3. Packages Overview
4. rplidar_ros
5. mapping_bringup
6. leo_navigation_tutorial
7. nav2_wavefront_frontier_exploration
8. Installation
9. Running the Navigation System
10. Dependencies
11. Troubleshooting
12. Contributors

---

# Folder Structure

```
Navigation/
│
├── rplidar_ros/
│
├── mapping_bringup/
│
├── leo_navigation_tutorial/
│
├── nav2_wavefront_frontier_exploration/
│
└── README.md
```

Each package performs a **specific role within the navigation pipeline**.

---

# Navigation Architecture

The robot navigation pipeline works as follows:

```
RPLidar Sensor
      │
      ▼
Laser Scan Topic (/scan)
      │
      ▼
SLAM Toolbox
      │
      ▼
Occupancy Grid Map
      │
      ▼
AMCL Localization
      │
      ▼
Nav2 Planner + Controller
      │
      ▼
Robot Motion (/cmd_vel)
```

For autonomous exploration:

```
Occupancy Grid Map
      │
      ▼
Frontier Detection
      │
      ▼
Goal Generation
      │
      ▼
Nav2 Navigation
```

---

# Package Overview

| Package                               | Purpose                                          |
| ------------------------------------- | ------------------------------------------------ |
| `rplidar_ros`                         | ROS2 driver for the RPLidar sensor               |
| `mapping_bringup`                     | Launch system to start mapping, NAV2 and RPlidar |
| `leo_navigation_tutorial`             | Core Nav2 configuration and navigation setup     |
| `nav2_wavefront_frontier_exploration` | Frontier exploration algorithm                   |

---

# rplidar_ros

External Repository used: 
Repository cloned using:

  git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git

## Purpose

This package provides the **ROS2 driver for the RPLidar a2m12 sensor**, allowing the robot to collect **laser scan data** used for mapping and obstacle detection.

### Important Files

```
rplidar_ros/
│
├── launch/
│   rplidar_a1_launch.py
│   view_rplidar_a1_launch.py
│
├── src/
│   rplidar_node.cpp
│   rplidar_client.cpp
│
├── include/
│   rplidar_node.hpp
│
└── sdk/
```

### Key Functionality

* Communicates with RPLidar hardware
* Publishes laser scan data to:

```
/scan
```

* Supports multiple LiDAR models (A1, A2, A3, S1, S2 etc.)

### Modifications / Configuration

For this project:

* Configured **RPLidar A2M12 launch file**
* Integrated LiDAR output with **SLAM Toolbox**
* Added **RViz configuration for visualization**

Example launch:

```
ros2 launch rplidar_ros rplidar_a2m12_launch.py
```

---

# mapping_bringup

## Purpose

This package provides a **simple launch interface to start the mapping system**.

### Structure

```
mapping_bringup/
│
├── launch/
│   mapping_bringup.launch.py
│
├── mapping_bringup/
│   __init__.py
│
└── package.xml
```

### Function

The launch file initializes:

* Lidar driver
* SLAM toolbox
* Required mapping nodes

### Modifications

A **custom launch file was created** to simplify the mapping process.

Instead of launching multiple nodes manually, mapping can be started with:

```
ros2 launch mapping_bringup mapping_bringup.launch.py
```

This improves usability during testing and deployment.

---

# leo_navigation_tutorial

External Repository used: 
Repository cloned using:

  git clone -b ros2 https://github.com/LeoRover/leo_navigation_tutorial.git

## Purpose

This package contains the **core navigation configuration for the robot** using the **ROS2 Navigation Stack (Nav2)**.

It defines:

* Navigation parameters
* Localization settings
* SLAM configuration
* Behavior trees
* Map files

### Structure

```
leo_navigation_tutorial/

config/
│
├── amcl.yaml
├── navigation.yaml
├── slam_toolbox.yaml
├── navigation_stereo_camera.yaml
│
└── behavior_trees/
    navigate_w_replanning_time.xml

launch/
│
├── amcl.launch.py
├── navigation.launch.xml
└── slam_toolbox.launch.py

maps/
│
├── empty_map.yaml
└── empty_map.pgm
```

### Configuration Files

**amcl.yaml**

Defines parameters for robot localization including:

* particle filter settings
* sensor model
* update frequency

---

**navigation.yaml**

Contains Nav2 parameters:

* global planner
* local planner
* costmaps
* obstacle inflation
* controller settings

---

**slam_toolbox.yaml**

Configuration for **SLAM Toolbox**, responsible for building the environment map.

---

**behavior_trees/navigate_w_replanning_time.xml**

Defines the **navigation behaviour tree** used by Nav2.

It allows the robot to:

* replan paths dynamically
* recover from obstacles
* continue navigation when path changes

---

### Modifications

For this project the following adjustments were made:

* Tuned **navigation.yaml parameters** for robot movement speed
* Adjusted **costmap resolution** for better obstacle detection
* Configured **SLAM Toolbox parameters for RPLidar data**
* Added a **custom behavior tree for dynamic replanning**
* Configured launch files to simplify navigation startup

Example navigation launch:

```
ros2 launch leo_navigation_tutorial navigation.launch.xml
```

---

# nav2_wavefront_frontier_exploration

External Repository used: 
Repository cloned using:

  git clone https://github.com/SeanReg/nav2_wavefront_frontier_exploration.git
  
## Purpose

This package implements **Wavefront Frontier Detection (WFD)** for **autonomous exploration**.

It allows the robot to automatically **discover unexplored regions of the map**.

### Structure

```
nav2_wavefront_frontier_exploration/

nav2_wfd/
│
├── wavefront_frontier.py
└── __init__.py
```

### Frontier Exploration Concept

Frontiers are boundaries between:

```
Known space
and
Unknown space
```

The algorithm:

1. Reads the occupancy grid map
2. Detects frontier cells
3. Chooses the nearest frontier
4. Sends a navigation goal to Nav2

### Modifications

For this project:

* Integrated frontier goals with **Nav2 navigation system**
* Adjusted exploration parameters for indoor environments
* Optimized frontier selection to reduce unnecessary movement

Run exploration using:

```
ros2 run nav2_wfd explore
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

---

# Running the Navigation System

### 1 Start SLAM Mapping

```
ros2 launch mapping_bringup mapping_bringup.launch.py
```

---

### 2 Start Autonomous Exploration

```
ros2 run nav2_wfd explore
```

---

# Dependencies

The navigation system depends on the following ROS2 packages:

```
nav2
slam_toolbox
rplidar_ros
tf2
geometry_msgs
sensor_msgs
nav_msgs
rviz2
```

Install dependencies using:

```
rosdep install --from-paths src --ignore-src -r -y
```

---

# Troubleshooting

### LiDAR not detected

Check available serial ports:

```
ls /dev/ttyUSB*
```

Update the launch file if necessary.

---

### No map generated

Check if laser scans are published:

```
ros2 topic echo /scan
```

---

### Robot not moving

Check command velocity topic:

```
ros2 topic echo /cmd_vel
```

---

# Contributors

## Contributors

- [S Vishnuvardha Rajeshwaran](https://github.com/vishnurajeshwaran)

University of Manchester - Autonomous Robotics Project

---

# License

Each package contains its own license file.
Refer to the individual package directories for licensing details.

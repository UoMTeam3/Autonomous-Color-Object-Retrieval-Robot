#  Autonomous Color Object Retrieval Robot (Leo Rover)

## Overview
This repository contains the implementation of an **Autonomous Color Object Retrieval System** built on the **Leo Rover platform**. The robot is designed to autonomously detect, navigate to, pick up, and sort objects based on their color.

The system integrates:
-  Computer Vision for object detection  
-  Autonomous Navigation for movement and mapping  
-  Robotic Manipulation for object interaction  
-  Mechanical Design for hardware integration  

---

##  Project Objectives
The main goals of this project are to:

- Detect objects using lightweight Neural Networks   
- Navigate autonomously within an unknown environment  
- Pick up objects using a robotic arm  
- Sort and place objects into designated locations  
- Operate fully autonomously without human intervention  

---

---

##  Repository Structure
├── ComputerVision/
│ ├── object_detection/
│ ├── object_detection_interface/
│ └── README.md
│
├── Navigation/
│ ├── rplidar_ros/
│ ├── mapping_bringup/
│ ├── leo_navigation_tutorial/
│ ├── nav2_wavefront_frontier_exploration/
│ └── README.md
│
├── Manipulator/
│ ├── MyCobot_ros2_team3/
│ │ ├── mycobot_280/
│ │ │ └── mycobot_280pi/
│ │ │ └── mycobot_280pi/
│ │ └── README.md
│ └── README.md
│
├── Design/
│ └── (CAD files and hardware designs)
│
└── README.md
---

## ⚙️ Technologies Used

### Software
- Python  
- ROS2  
- YOLOV8 
- Nav2  
- slam 
### Hardware
- Leo Rover  
- RPLiDAR  
- RGB Camera 
- MyCobot 280 Robotic Arm  

---

## 📦 Installation

### Prerequisites
- Ubuntu 20.04 / 22.04  
- ROS2 jazzy  
- Python 3  

### Clone Repository
git clone https://github.com/UoMTeam3/Autonomous-Color-Object-Retrieval-Robot.git
cd Autonomous-Color-Object-Retrieval-Robot

---

# Contributors
- [Benjamin Woodruff](https://github.com/Ben-Woodruff)
- [Si Qin](https://github.com/god-good-debug)
- [Zicheng Hao](https://github.com/maxclementine21-boop)
- [Po Cheng Chen](https://github.com/cpcinUoM)
- [S Vishnuvardha Rajeshwaran](https://github.com/vishnurajeshwaran)

University of Manchester - Autonomous Robotics Project

---

# License

Refer to the individual package licenses within the repository.



# Computer Vision System

This folder contains the **computer vision system used for object detection and coordinate frame transforms from camera_link frame to map frame** on the Leo Rover platform. The system uses **YOLO-based object detection integrated with ROS2** to identify colored objects in the environment and publish their positions so the robot can navigate toward them.

The computer vision pipeline allows the robot to:

* Detect objects using a trained YOLO model
* Publish detected object information to ROS2 topics
* Convert camera detections into robot coordinate frames
* Provide navigation targets for the robot

---

# Table of Contents

1. Folder Structure
2. System Architecture
3. Packages Overview
4. Object Detection Package
5. Object Detection Interface
6. Custom ROS Messages
7. YOLO Model
8. Running the System
9. Dependencies
10. Troubleshooting
11. Contributors

---

# Folder Structure

```
ComputerVision/
│
├── object_detection/
│
├── object_detection_interface/
│
└── README.md
```

The computer vision module consists of **two ROS2 packages**:

| Package                      | Purpose                                        |
| ---------------------------- | ---------------------------------------------- |
| `object_detection`           | YOLO-based object detection node               |
| `object_detection_interface` | Custom ROS2 messages for object detection data |

---

# System Architecture

The computer vision pipeline works as follows:

```
Camera Image
     │
     ▼
YOLO Object Detection
     │
     ▼
Object Publisher Node
     │
     ▼
Object Detection Messages
     │
     ▼
Coordinate Transformation
     │
     ▼
Navigation Target
```

The system detects objects from camera images and publishes their **coordinates relative to the robot frame** so that navigation nodes can move toward them.

---

# Package Overview

| Package                      | Description                                                |
| ---------------------------- | ---------------------------------------------------------- |
| `object_detection`           | Runs the YOLO object detection model and publishes results |
| `object_detection_interface` | Defines custom ROS messages for object detection           |

---

# object_detection Package

## Purpose

This package implements the **core object detection functionality** using a trained **YOLO model**.

It processes camera images and publishes detected object data.

---

## Structure

```
object_detection/

launch/
│
└── object_detection_launch.py

object_detection/
│
├── object_publisher.py
├── navigate_to_block.py
├── tf2_transform.py
├── best.pt
└── __init__.py

test/

setup.py
setup.cfg
package.xml
```

---

## Key Files

### best.pt

This is the **trained YOLO model** used for object detection.

The model has been trained to detect the **colored objects used in the project**.

---

### object_publisher.py

This node performs **object detection using the YOLO model**.

Responsibilities:

* Load trained YOLO model (`best.pt`)
* Subscribe to camera images
* Run object detection inference
* Publish detected object information

Published data typically includes:

* Object class
* Bounding box
* Object coordinates

---

### tf2_transform.py

This node converts detected object coordinates between coordinate frames using **ROS2 TF2**.

Purpose:

* Transform camera frame coordinates
* Convert them into the robot’s base frame
* Ensure navigation goals are accurate

---

### navigate_to_block.py

This script allows the robot to **navigate toward detected objects**.

Functionality:

* Receives object detection messages
* Determines the target object position
* Sends navigation goals to the robot 

This integrates the **computer vision module with the navigation stack**.

---

# Launch File

### object_detection_launch.py

This launch file starts the computer vision pipeline.

It typically launches:

* object detection node
* coordinate transformation node

Example:

```
ros2 launch object_detection object_detection_launch.py
```

---

# object_detection_interface Package

## Purpose

This package defines **custom ROS2 message types** used to communicate object detection information between nodes.

---

## Structure

```
object_detection_interface/

msg/
│
├── YoloInterface.msg
└── ObjectCoordinates.msg

include/
src/

package.xml
CMakeLists.txt
```

---

# Custom ROS Messages

## YoloInterface.msg

This message is used to transmit **YOLO detection results**.

Typical information includes:

* detected object label
* bounding box position
* confidence score

---

## ObjectCoordinates.msg

This message contains the **position of detected objects relative to the robot**.

Typical fields include:

* object x coordinate
* object y coordinate
* object distance
* object label

These coordinates allow the robot to **navigate to the detected object**.

---

# YOLO Object Detection Model

The detection model used is stored as:

```
best.pt
```

This file contains the **trained YOLO weights** used to identify objects.

The model is loaded by the detection node and used to perform **real-time inference on camera images**.

---

# Running the Computer Vision System

After building the ROS2 workspace, run:

```
colcon build
source install/setup.bash
```

Start the object detection node:

```
ros2 launch object_detection object_detection_launch.py
```

Once running, the system will:

1. Receive camera images
2. Detect objects using YOLO
3. Publish object detection results
4. Transform coordinates
5. Send navigation targets

---

# Dependencies

The computer vision system depends on:

```
ROS2
OpenCV
PyTorch
YOLO
cv_bridge
sensor_msgs
geometry_msgs
tf2_ros
```

Make sure all dependencies are installed before building the workspace.

---

# Troubleshooting

### Model not loading

Check that the YOLO model exists:

```
object_detection/object_detection/best.pt
```

---

### No objects detected

Possible causes:

* camera topic not publishing
* incorrect image topic name
* low detection confidence threshold

---

### TF transformation errors

Check that the TF tree is running:

```
ros2 run tf2_tools view_frames
```

---

# Contributors



# License

Refer to the individual package licenses within the repository.

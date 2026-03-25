#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from object_detection_interface.msg import ObjectCoordinates
from object_detection_interface.msg import YoloInterface
from ultralytics import YOLO
#import pyrealsense2 as rs
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo 
from cv_bridge import CvBridge

MODEL_PATH = "/home/ben/ros_ws/src/object_detection/object_detection/best.pt"
CONF_THRES = 0.5
IMG_SIZE = 640
DEPTH_WIN = 5

class object_publisherNode(Node):
    """A ROS2 Node that publishes an amazing quote."""

    def __init__(self):
        super().__init__('object_publisher_node')

        self.model = YOLO(MODEL_PATH)
        self.get_logger().info("YOLOv8 model loaded")
        self.target_locked = False 
        self.latest_yolo_msg = None
        
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.bridge = CvBridge()
        self.intrinsics = None
        self.depth_image = None

        #subcriber to colour image 
        self.camera_subscriber = self.create_subscription(
            msg_type=Image, 
            topic='/camera/camera/color/image_raw', # check topic 
            callback=self.color_callback, 
            qos_profile=1
        )

        self.camera_info_sub = self.create_subscription(
            msg_type=CameraInfo,
            topic='/camera/camera/color/camera_info',
            callback=self.camera_info_callback,
            qos_profile=1
        )

        # Subscribe to ALIGNED DEPTH image
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',  #check topic 
            self.depth_callback,
            1
        )

        self.object_publisher = self.create_publisher(
            msg_type=YoloInterface,
            topic='/block_coordinates',
            qos_profile=1)
        
        self.publish_timer = self.create_timer(
            1.0, 
            self.publish_objects
        )
        
    def camera_info_callback(self, msg):
        if self.fx is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.get_logger().info("Camera intrinsics received")

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def color_callback(self, msg):

        if self.depth_image is None:
            return

        if self.fx is None:
            return
        
        color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run YOLO
        results = self.model.predict(
            source=color_image,
            conf=CONF_THRES,
            imgsz=IMG_SIZE,
            verbose=False
        )

        yolo_msg = YoloInterface()
        yolo_msg.header.stamp = self.get_clock().now().to_msg()
        yolo_msg.header.frame_id = "camera_color_optical_frame"

        for r in results:
            if r.boxes is None:
                continue

            for box in r.boxes:

                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                cls_id = int(box.cls[0].item())
                class_name = self.model.names.get(cls_id, str(cls_id))

                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                # Depth window sampling
                win = DEPTH_WIN
                half = win // 2

                x_start = max(cx - half, 0)
                x_end = min(cx + half, self.depth_image.shape[1] - 1)
                y_start = max(cy - half, 0)
                y_end = min(cy + half, self.depth_image.shape[0] - 1)

                depth_patch = self.depth_image[y_start:y_end + 1, x_start:x_end + 1].astype(float)
                valid = depth_patch[depth_patch > 0]

                if valid.size == 0:
                    continue

                depth_m = np.mean(valid) * 0.001  # depth in mm → meters

                # Get intrinsics once
                Z = depth_m
                X = (cx - self.cx) * Z / self.fx
                Y = (cy - self.cy) * Z / self.fy
                #fix the orange block problem
                HEIGHT_THRESHOLD = 0.12 

                if Y > HEIGHT_THRESHOLD:
                    self.get_logger().info(f"ignore : Y={Y:.2f} too close to ground, not block")
                    continue

                obj = ObjectCoordinates()
                obj.class_name = class_name
                obj.x = float(X)
                obj.y = float(Y)
                obj.z = float(Z)
                obj.depth = float(depth_m)

                yolo_msg.yolo_interface.append(obj)

                self.get_logger().info(
                    f"{class_name} → X:{X:.2f} Y:{Y:.2f} Z:{Z:.2f}"
                )

        self.latest_yolo_msg = yolo_msg
        self.target_locked = True
    def publish_objects(self):
        if self.latest_yolo_msg is not None:
            self.object_publisher.publish(self.latest_yolo_msg)

def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure
    certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)

        object_publisher = object_publisherNode()

        rclpy.spin(object_publisher)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()

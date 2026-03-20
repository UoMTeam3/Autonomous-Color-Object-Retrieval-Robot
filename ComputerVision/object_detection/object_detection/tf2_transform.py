import rclpy
from rclpy.node import Node

import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException

from geometry_msgs.msg import PointStamped
from object_detection_interface.msg import YoloInterface
from object_detection_interface.msg import ObjectCoordinates
import math

class TF2ListenerNode(Node):

    def __init__(self):
        super().__init__('tf2_listener_node')

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribe to detected object coordinates
        self.object_sub = self.create_subscription(
            YoloInterface,
            '/block_coordinates',
            self.object_callback,
            1
        )

        self.map_coordinate_publisher = self.create_publisher(
            msg_type=YoloInterface,
            topic='/block_map_coordinates',
            qos_profile=1)
        
        # Track already processed classes
        self.sent_classes = set()

    def object_callback(self, msg):

        if len(msg.yolo_interface) == 0:
            return
        
        transformed_objects = []

        for obj in msg.yolo_interface:
            
            #  Skip if this class already sent
            if obj.class_name in self.sent_classes:
                continue

            # Create point in camera frame
            point_camera = PointStamped()
            point_camera.header.frame_id = "camera_color_optical_frame"
            point_camera.header.stamp = rclpy.time.Time().to_msg()
            point_camera.point.x = obj.x
            point_camera.point.y = obj.y
            point_camera.point.z = obj.z

            try:
                
                tfs =   self.tf_buffer.lookup_transform(
                    "map", 
                    "camera_color_optical_frame", 
                    rclpy.time.Time()
                )

                # Transform to map frame
                point_map = tf2_geometry_msgs.do_transform_point(
                    point=point_camera, 
                    transform=tfs
                )

                # Compute distance from robot (map origin approx)
                dist = math.sqrt(
                    point_map.point.x**2 + point_map.point.y**2
                )

                transformed_objects.append({
                    "class": obj.class_name,
                    "x": point_map.point.x,
                    "y": point_map.point.y,
                    "z": point_map.point.z,
                    "distance": dist
                })

            except TransformException as e:
                self.get_logger().warn(f"Transform failed: {e}")

            #  No new objects
        if len(transformed_objects) == 0:
            return

        #  Pick closest object ONLY
        closest_obj = min(transformed_objects, key=lambda x: x["distance"])

        # Create message for ONLY ONE object
        map_msg = YoloInterface()
        map_object = ObjectCoordinates()

        map_object.class_name = closest_obj["class"]
        map_object.x = closest_obj["x"]
        map_object.y = closest_obj["y"]
        map_object.z = closest_obj["z"]

        map_msg.yolo_interface.append(map_object)

        #  Publish ONCE
        self.map_coordinate_publisher.publish(map_msg)

        #  Mark class as sent
        self.sent_classes.add(closest_obj["class"])

        self.get_logger().info(
            f"Sent goal for {closest_obj['class']} | "
            f"X={closest_obj['x']:.2f}, Y={closest_obj['y']:.2f}"
        )

def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)

        tf2_transform = TF2ListenerNode()

        rclpy.spin(tf2_transform)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()

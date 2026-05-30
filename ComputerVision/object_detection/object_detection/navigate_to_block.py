import rclpy
import math

from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from rclpy.task import Future

from std_msgs.msg import Bool
from object_detection_interface.msg import YoloInterface
from geometry_msgs.msg import Pose
from nav2_msgs.action import NavigateToPose

import tf2_ros
from tf2_ros import TransformException
from tf_transformations import euler_from_quaternion


class Nav2NavigateToPoseActionClient(Node):

    def __init__(self):
        super().__init__('nav2_navigate_to_pose_action_client')

        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

<<<<<<< HEAD
        self.goal_in_progress = False

        # Queue + tracking
        self.goal_queue = []
        self.visited_classes = set()
        self.current_goal_class = None
=======
       
        # Control flags
       
        self.block_already_detected = False
        self.nav_enabled = False
        self.last_detection = None
        self.goal_in_progress = False
        self.goal_handle = None
>>>>>>> 57d890f (updated)

       
        # Robot pose (from TF)
       
        self.robot_x = None
        self.robot_y = None
        self.yaw = None

       
        # TF listener
       
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

       
        # Queue + tracking
       
        self.goal_queue = []
        self.visited_classes = set()
        self.current_goal_class = None

        # publisher 

        self.block_detected_pub = self.create_publisher(
            Bool, 
            '/block_detected', 
            1)

        self.done_pub = self.create_publisher(
            Bool, 
            '/nav_done', 
            1)
       
        # Subscribers
       
        self.block_sub = self.create_subscription(
            YoloInterface,
            '/block_map_coordinates',
            self.block_callback,
            1
        )

        self.enable = self.create_subscription(
            Bool,
            '/nav_enable',
            self.nav_enable_callback,
            1
        )

   
    # NAV ENABLE CALLBACK (FIXED)
   
    def nav_enable_callback(self, msg):

        if msg.data and not self.nav_enabled:
            self.get_logger().info("Navigation ENABLED")

        self.nav_enabled = msg.data

        if self.nav_enabled:
            self.process_next_goal()
    
    # TF: Get robot pose
   
    def update_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

            self.robot_x = transform.transform.translation.x
            self.robot_y = transform.transform.translation.y

            q = transform.transform.rotation
            _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        except TransformException as ex:
            self.get_logger().warn(f"TF lookup failed: {ex}")
            self.robot_x = None
            self.robot_y = None
            self.yaw = None

   
    # BLOCK CALLBACK (FIXED)
   
    def block_callback(self, msg):

        # ALWAYS detect (even during exploration)
        if not self.block_already_detected:
            self.get_logger().info("First block detected!")

            self.block_detected_pub.publish(Bool(data=True))
            self.block_already_detected = True

        # Only proceed with navigation logic if enabled
        # if not self.nav_enabled:
        #     return

        self.update_robot_pose()

        if len(msg.yolo_interface) == 0:
            return

        for obj in msg.yolo_interface:

            class_name = obj.class_name
<<<<<<< HEAD

            # Skip if already visited
            if class_name in self.visited_classes:
                continue

            x = obj.x - 0.3 # 30 cm offset 
            y = obj.y

            #  Add to queue if not already queued
            if not any(g[2] == class_name for g in self.goal_queue):
                self.goal_queue.append((x, y, class_name))
                self.get_logger().info(f"Queued goal for {class_name}")

        #  Try to process queue
        self.process_next_goal()

    def process_next_goal(self):

        if self.goal_in_progress:
            return

        if len(self.goal_queue) == 0:
            return

        x, y, class_name = self.goal_queue.pop(0)
=======

            # Skip visited
            if class_name in self.visited_classes:
                continue

            # Skip if already queued
            if any(g[2] == class_name for g in self.goal_queue):
                continue

            if self.robot_x is not None and self.robot_y is not None:

                dx = obj.x - self.robot_x
                dy = obj.y - self.robot_y
                dist = math.sqrt(dx**2 + dy**2)

                if dist > 1e-6:
                    goal_yaw = math.atan2(
                        obj.y - self.robot_y,
                        obj.x - self.robot_x
                    )

                    offset = 0.3

                    x = obj.x - offset * math.cos(goal_yaw)
                    y = obj.y - offset * math.sin(goal_yaw)
                else:
                    x = obj.x
                    y = obj.y
                    goal_yaw = 0.0
            else:
                x = obj.x
                y = obj.y
                goal_yaw = 0.0

            self.goal_queue.append((x, y, class_name, goal_yaw))
            self.get_logger().info(f"Queued goal for {class_name}")

        self.process_next_goal()
        if not self.nav_enabled:
            return

   
    # PROCESS QUEUE
   
    def process_next_goal(self):

        if not self.nav_enabled:
            return

        if self.goal_in_progress:
            return

        if len(self.goal_queue) == 0:
            return

        x, y, class_name, goal_yaw = self.goal_queue.pop(0)
>>>>>>> 57d890f (updated)

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
<<<<<<< HEAD
        pose.orientation.w = 1.0
=======

        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = math.sin(goal_yaw / 2.0)
        pose.orientation.w = math.cos(goal_yaw / 2.0)
>>>>>>> 57d890f (updated)

        self.current_goal_class = class_name

        self.send_goal_async(pose, "")

<<<<<<< HEAD
=======
   
    # SEND GOAL
   
>>>>>>> 57d890f (updated)
    def send_goal_async(self, desired_pose: Pose, behaviour_tree: str) -> None:

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose = desired_pose
        goal_msg.behavior_tree = behaviour_tree

        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for Nav2 action server...')

        self.get_logger().info(
            f"Sending goal ({self.current_goal_class}) "
            f"x:{desired_pose.position.x:.2f}, y:{desired_pose.position.y:.2f}"
        )

        self.goal_in_progress = True

        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.action_feedback_callback
        )

        self.send_goal_future.add_done_callback(self.goal_response_callback)

   
    # GOAL RESPONSE 
   
    def goal_response_callback(self, future: Future) -> None:

        goal: ClientGoalHandle = future.result()

        if not goal.accepted:
            self.get_logger().info('Goal rejected')
            self.goal_in_progress = False
            return

        self.get_logger().info('Goal accepted')

        # Store goal handle for cancellation
        self.goal_handle = goal

        self.get_result_future = goal.get_result_async()
        self.get_result_future.add_done_callback(self.action_result_callback)

   
    # RESULT CALLBACK
   
    def action_result_callback(self, future: Future) -> None:

        self.get_logger().info('Navigation finished')

        self.goal_in_progress = False

<<<<<<< HEAD
        # Mark class as completed
=======
>>>>>>> 57d890f (updated)
        if self.current_goal_class:
            self.visited_classes.add(self.current_goal_class)
            self.get_logger().info(f"Completed {self.current_goal_class}")

        self.current_goal_class = None
<<<<<<< HEAD

        # Process next goal

=======
        self.done_pub.publish(Bool(data=True))

>>>>>>> 57d890f (updated)
        self.process_next_goal()

   
    # FEEDBACK
   
    def action_feedback_callback(self, feedback_msg: NavigateToPose.Feedback) -> None:

        feedback = feedback_msg.feedback

        self.get_logger().info(
            f'Distance remaining: {feedback.distance_remaining:.2f}'
        )

def main(args=None):

    try:
        rclpy.init(args=args)

        node = Nav2NavigateToPoseActionClient()

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()

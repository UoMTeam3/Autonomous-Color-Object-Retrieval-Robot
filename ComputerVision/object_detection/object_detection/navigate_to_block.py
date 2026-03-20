import rclpy
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from rclpy.task import Future

from object_detection_interface.msg import YoloInterface
from geometry_msgs.msg import Pose, PointStamped
from nav2_msgs.action import NavigateToPose


class Nav2NavigateToPoseActionClient(Node):

    def __init__(self):
        super().__init__('nav2_navigate_to_pose_action_client')

        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.goal_in_progress = False

        # Queue + tracking
        self.goal_queue = []
        self.visited_classes = set()
        self.current_goal_class = None

        # subscribe to block coordinates in map frame
        self.block_sub = self.create_subscription(
            YoloInterface,
            '/block_map_coordinates',
            self.block_callback,
            1
        )

    def block_callback(self, msg):

        if len(msg.yolo_interface) == 0:
            return

        for obj in msg.yolo_interface:

            class_name = obj.class_name

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

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.orientation.w = 1.0

        self.current_goal_class = class_name

        self.send_goal_async(pose, "")

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

    def goal_response_callback(self, future: Future) -> None:

        goal: ClientGoalHandle = future.result()

        if not goal.accepted:
            self.get_logger().info('Goal rejected')
            self.goal_in_progress = False
            return

        self.get_logger().info('Goal accepted')

        self.get_result_future = goal.get_result_async()
        self.get_result_future.add_done_callback(self.action_result_callback)

    def action_result_callback(self, future: Future) -> None:

        self.get_logger().info('Navigation finished')

        self.goal_in_progress = False

        # Mark class as completed
        if self.current_goal_class:
            self.visited_classes.add(self.current_goal_class)
            self.get_logger().info(f"Completed {self.current_goal_class}")

        self.current_goal_class = None

        # Process next goal

        self.process_next_goal()

    def action_feedback_callback(self, feedback_msg: NavigateToPose.Feedback) -> None:

        feedback = feedback_msg.feedback

        self.get_logger().info(
            f'Distance remaining: {feedback.distance_remaining:.2f}'
        )

def main(args=None):

    try:
        rclpy.init(args=args)

        navigate_to_block = Nav2NavigateToPoseActionClient()

        rclpy.spin(navigate_to_block)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()

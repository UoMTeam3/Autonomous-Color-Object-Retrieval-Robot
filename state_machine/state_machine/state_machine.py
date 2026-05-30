#!/usr/bin/env python3

import rclpy
from enum import Enum

from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

import tf2_ros
from tf2_ros import TransformException


class State(Enum):
    INIT = 0
    EXPLORE = 1
    GO_TO_BLOCK = 2
    WAIT = 3
    RETURN_HOME = 4
    DONE = 5


class NavigationStateMachine(Node):

    def __init__(self):
        super().__init__('navigation_state_machine')

        self.state = State.INIT

        # Nav2 client (for return home only)
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.initial_pose = None
        self.wait_start_time = None

        # Flags
        self.block_handled = False

        # Publishers
        self.explore_pub = self.create_publisher(Bool, '/explore_enable', 1)
        self.nav_enable_pub = self.create_publisher(Bool, '/nav_enable', 1)

        # Subscribers
        self.create_subscription(
            Bool,
            '/block_detected',
            self.block_detected_callback,
            1
        )

        self.create_subscription(
            Bool,
            '/nav_done',
            self.nav_done_callback,
            1
        )

        # Timer
        self.timer = self.create_timer(0.5, self.run_state_machine)

    # -------------------------
    # TF: get robot pose
    # -------------------------
    def get_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.orientation = transform.transform.rotation

            return pose

        except TransformException:
            return None

    # -------------------------
    # BLOCK DETECTED
    # -------------------------
    def block_detected_callback(self, msg):

        if msg.data and self.state == State.EXPLORE:
            self.get_logger().info("Block detected → stopping exploration")

            self.explore_pub.publish(Bool(data=False))
            self.nav_enable_pub.publish(Bool(data=True))

            self.state = State.GO_TO_BLOCK

    # -------------------------
    # NAV DONE (from nav node)
    # -------------------------
    def nav_done_callback(self, msg):

        if msg.data and self.state == State.GO_TO_BLOCK:
            self.get_logger().info("Reached block → waiting")

            self.wait_start_time = self.get_clock().now()
            self.state = State.WAIT

    # -------------------------
    # SEND GOAL (return home)
    # -------------------------
    def send_goal(self, pose):

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Waiting for Nav2...")

        self.get_logger().info("Returning to home")

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("Return goal rejected")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_done_callback)

    def goal_done_callback(self, future):

        self.get_logger().info("Reached home")
        self.state = State.DONE

    # -------------------------
    # MAIN FSM
    # -------------------------
    def run_state_machine(self):

        # INIT
        if self.state == State.INIT:

            pose = self.get_robot_pose()

            if pose is not None:
                self.initial_pose = pose
                self.get_logger().info("Initial pose stored")

                self.explore_pub.publish(Bool(data=True))
                self.nav_enable_pub.publish(Bool(data=False))

                self.state = State.EXPLORE

        # EXPLORE
        elif self.state == State.EXPLORE:
            pass

        # GO TO BLOCK
        elif self.state == State.GO_TO_BLOCK:
            pass

        # WAIT at block
        elif self.state == State.WAIT:

            elapsed = (self.get_clock().now() - self.wait_start_time).nanoseconds / 1e9

            if elapsed > 5.0:
                self.send_goal(self.initial_pose)
                self.state = State.RETURN_HOME

        # RETURN HOME
        elif self.state == State.RETURN_HOME:
            pass

        # DONE
        elif self.state == State.DONE:
            self.get_logger().info("Mission complete")

            self.nav_enable_pub.publish(Bool(data=False))
            self.explore_pub.publish(Bool(data=False))

            # Stop FSM cleanly
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = NavigationStateMachine()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
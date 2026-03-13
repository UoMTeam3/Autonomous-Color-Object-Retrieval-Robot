import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from  geometry_msgs.msg import Point32
import os
import time
import math
import pymycobot
from pymycobot.genre import Coord
from packaging import version

# min low version require
MIN_REQUIRE_VERSION = '4.0.0'

current_verison = pymycobot.__version__
print('current pymycobot library version: {}'.format(current_verison))
if version.parse(current_verison) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError('The version of pymycobot library must be greater than {} or higher. The current version is {}. Please upgrade the library version.'.format(
        MIN_REQUIRE_VERSION, current_verison))
else:
    print('pymycobot library version meets the requirements!')
    from pymycobot import MyCobot280


class Slider_Subscriber(Node):
    def __init__(self):
        super().__init__("control_slider")
        self.subscription = self.create_subscription(
             Point32,
             "Do_Not_Read",
             self.listener_callback,
             10
         )
        self.subscription

        self.subscription2 = self.create_subscription(
             Point32,
             "Bin_Topic",
             self.bin_listener_callback,
             10
         )
        self.subscription2

        pub = self.create_publisher(
            msg_type=JointState,
            topic="joint_states",
            qos_profile=10
        )

        rate = self.create_rate(30)

        # pub joint state
        joint_state_send = JointState()
        joint_state_send.header = Header()

        joint_state_send.name = [
            "joint2_to_joint1",
            "joint3_to_joint2",
            "joint4_to_joint3",
            "joint5_to_joint4",
            "joint6_to_joint5",
            "joint6output_to_joint6",
        ]
        joint_state_send.velocity = [0.0, ]
        joint_state_send.effort = []

        # self.robot_m5 = os.popen("ls /dev/ttyUSB*").readline()[:-1]
        # self.robot_wio = os.popen("ls /dev/ttyACM*").readline()[:-1]
        # if self.robot_m5:
        #     port = self.robot_m5
        # else:
        #     port = self.robot_wio
        self.declare_parameter('port', '/dev/ttyAMA0')
        self.declare_parameter('baud', 1000000)
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.get_logger().info("port:%s, baud:%d" % (port, baud))
        self.mc = MyCobot280(port, baud)
        time.sleep(0.05)
        self.mc.set_fresh_mode(1)
        time.sleep(0.05)
        self.theta_offset = 0

        # Get the current head coordinates and posture
        coords = self.mc.get_coords()
        print(coords)
        # # Intelligently plan the route, so that the head reaches the coordinates [57.0, -107.4, 316.3] in a linear manner, and maintains the posture [-93.81, -12.71, -163.49], with a speed of 80mm/s
        self.mc.send_coords([57.0, -107.4, 316.3, -93.81, -12.71, -163.49], 80, 1)

        # Set the waiting time to 1.5 seconds
        time.sleep(1.5)

        # Intelligently plan the route, let the head reach the coordinates [-13.7, -107.5, 223.9] in a linear manner, and maintain the posture [165.52, -75.41, -73.52], with a speed of 80mm/s
        self.mc.send_coords([-13.7, -107.5, 223.9, 165.52, -75.41, -73.52], 80, 1)

        # Set the waiting time to 1.5 seconds
        time.sleep(1.5)

        # self.get_logger().info('data_list: {} gripper_value: {} '.format(data_list, gripper_value))
        self.mc.set_gripper_value(0, 80, gripper_type=1)

        while rclpy.ok():
            rclpy.spin_once(self)
            joint_state_send.header.stamp = self.get_clock().now().to_msg()

            try:
                angles = self.mc.get_radians()
                data_list = []
                for _, value in enumerate(angles):
                    data_list.append(value)

                # self.get_logger().info('angles: {}'.format([round(math.degrees(angle), 2) for angle in data_list]))
                joint_state_send.position = data_list

                pub.publish(joint_state_send)

                rate.sleep()
            except Exception as e:
                print(e)

    def listener_callback(self, msg):
        #print(msg.x)
        #print(msg.y)
        #print(msg.z)
        self.mc.set_gripper_value(100, 80, gripper_type=1)
        time.sleep(1.5)
        self.mc.send_coords([msg.x,msg.y,msg.z+50,180, 0,self.theta_offset], 80, 1)
        time.sleep(1.5)
        self.mc.send_coords([msg.x,msg.y,msg.z,180, 0,self.theta_offset], 80, 1)
        time.sleep(1.5)
        coords = self.mc.get_coords()
        self.mc.set_gripper_value(0, 80, gripper_type=1)
        time.sleep(1.5)
        self.mc.send_coord(Coord.Z.value, msg.z+50, 70)
        time.sleep(1.5)
        grasped = self.mc.get_gripper_value()
        print(grasped)
        if grasped <= 30:
            print("Not Grasped!")
            self.theta_offset = ((self.theta_offset + 90 )% 180) 
            self.listener_callback(msg)
            return None
        elif grasped >= 20:
            print("Grasped")
            self.theta_offset = 0
        else:
            print("Uncertain")
        time.sleep(1.5)
        print(self.mc.get_gripper_value())
        #self.mc.send_coord(Coord.Z.value, msg.z, 70)
        time.sleep(1.5)
        #self.mc.set_gripper_value(100, 80, gripper_type=1)

    def bin_listener_callback(self, msg):
        #print(msg.x)
        #print(msg.y)
        #print(msg.z)
        self.mc.send_coords([msg.x,msg.y,msg.z,180, 0,0], 80, 1)
        time.sleep(1.5)
        self.mc.set_gripper_value(100, 80, gripper_type=1)
        

def main(args=None):
    rclpy.init(args=args)
    slider_subscriber = Slider_Subscriber()


    rclpy.spin(slider_subscriber)    

    slider_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

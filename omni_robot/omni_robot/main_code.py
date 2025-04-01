#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from dc_gamepad_msgs.msg import GamePad
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from custom_messages.msg import DigitalAndAnalogFeedback
from math import atan2
from std_msgs.msg import Bool
import time

def sign(x):
    if x >= 0.0:
        return 1.0
    else:
        return -1.0

class MainNode(Node):
    def __init__(self):
        super().__init__('main_node')
        self.init_publisher()
        self.init_subscriber()
        self.init_robot_variables()
        self.initialize()

    def initialize(self):
        self.reset_publisher.publish(Bool())
        offset = Vector3()
        offset.x = 0.60 / 2
        offset.y = -0.682 / 2
        self.odometry_offset_publisher.publish(offset)

    def init_publisher(self):
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.reset_publisher = self.create_publisher(Bool, '/reset_odometry', 10)
        self.odometry_offset_publisher = self.create_publisher(Vector3, 'offset_odometry', 10)

    def init_subscriber(self):
        self.gamepad_subscription = self.create_subscription(GamePad, '/pad', self.gamepad_callback, 10)
        self.sensor_subscription = self.create_subscription(DigitalAndAnalogFeedback, '/digital_analog_feedback', self.sensor_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

    def sensor_callback(self, msg):
        self.range_finder_x = msg.analog1_value * 10.69066089 + 0.208
        self.range_finder_y = msg.analog2_value * 10.704653505 + 0.208

        #print(f"{self.range_finder_x:.2f}, {self.range_finder_y:.2f}")

    def init_robot_variables(self):
        self.max_V = 3.0
        self.max_Wz = 3.0

        self.Vx = 0.0
        self.Vy = 0.0
        self.Wz = 0.0

        self.button_y = False
        self.previous_button_y = False

    def gamepad_callback(self, msg):
        self.Vx = (msg.left_analog_x - 128) / 128 * self.max_V
        self.Vy = -(msg.left_analog_y - 128) / 128 * self.max_V
        self.Wz = -(msg.right_analog_x - 128) / 128 * self.max_Wz

        self.button_y = msg.button_y
        self.previous_button_y = msg.previous_button_y

        print(self.Vx, self.Vy)

    def odom_callback(self, msg):
        if self.button_y and not self.previous_button_y:
            self.initialize()

        orientation = msg.pose.pose.orientation
        w, x, y, z = orientation.w, orientation.x, orientation.y, orientation.z

        yaw = atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))

        desired_yaw = 0.0
        error_yaw = desired_yaw - yaw
        self.Wz = 10.0 * error_yaw

        if abs(self.Wz) > 3.0:
            self.Wz = sign(self.Wz) * 3.0

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = self.Vx
        cmd_vel_msg.linear.y = self.Vy
        cmd_vel_msg.angular.z = self.Wz

        self.cmd_vel_publisher.publish(cmd_vel_msg)

        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        #print(f"{x:.2f}, {y:.2f}")

def main(args=None):
    rclpy.init(args=args)
    main_node = MainNode()
    rclpy.spin(main_node)
    main_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


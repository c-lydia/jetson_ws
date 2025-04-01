#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from custom_messages.msg import MotorCommand
from geometry_msgs.msg import Twist
from math import sqrt

class CmdVelNode(Node):
    def __init__(self):
        super().__init__('body_velocity_node')
        self.l = 66.2 * 0.01 / 2
        self.r = 127.0 * 0.001 / 2
        
        self.a = sqrt(2) / 2 / self.r
        self.b = self.l / self.r

        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.motor_publisher = self.create_publisher(MotorCommand, '/publish_motor', 10)
        
    def inverse_kinematics(self, Vx, Vy, omega_z):
        wheel_speed = [
            self.a*(Vy+Vx) - self.b * omega_z,
            self.a*(Vy-Vx) + self.b * omega_z,
            self.a*(Vy+Vx) + self.b * omega_z,
            self.a*(Vy-Vx) - self.b * omega_z
        ]
        return wheel_speed

    def cmd_vel_callback(self, msg):
        Vx = msg.linear.x
        Vy = msg.linear.y
        omega_z = msg.angular.z

        wheel_speeds = self.inverse_kinematics(Vx, Vy, omega_z)
        
        for i in range(4):
            MotorMsg = MotorCommand()
            MotorMsg.speedmode = True
            MotorMsg.can_id = i+1
            MotorMsg.goal = wheel_speeds[i]
            self.motor_publisher.publish(MotorMsg)

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_node = CmdVelNode()
    rclpy.spin(cmd_vel_node)
    cmd_vel_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
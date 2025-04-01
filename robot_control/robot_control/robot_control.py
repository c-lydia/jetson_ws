import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import Twist
from dc_gamepad_msgs.msg import GamePad
from sensor_msgs.msg import Imu
import math

class RobotControl(Node): 
    def __init__(self):
        super().__init__('robot_control')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pad_subscriber = self.create_subscription(GamePad, '/pad', self.gamepad_callback, 10)
        self.imu_subscriber = self.create_subscription(Imu, '/imu/data_raw', self.imu_callback, 10)
        self.desired_yaw = 0.0
        self.wz = 0.0
        self.kp = 1.0
        self.yaw_start = None
        self.previous_yaw = 0.0
        self.V_max = 1.0
        self.Wz_max = 1.0
        self.angular_velocity = 0.0
        self.cmd_vel_msg = Twist()

    def counter(self): 
        if self.wz > 2 * math.pi: 
            counter -= 1
        elif self.wz < -2 * math.pi: 
            counter += 1

    def imu_callback(self, imu_msg): 
        qx = imu_msg.orientation.x 
        qy = imu_msg.orientation.y 
        qz = imu_msg.orientation.z
        qw = imu_msg.orientation.w

        current_yaw = math.atan2(2.0*(qy*qz + qx*qw), 1 - 2*(qy**2 + qz**2))

        if self.yaw_start == None: 
            self.yaw_start = current_yaw 

        current_yaw -= self.yaw_start
        self.desired_yaw += (self.angular_velocity * 0.01)/128
        self.previous_yaw = current_yaw
        delta_yaw = current_yaw - self.previous_yaw

        if delta_yaw > math.pi: 
            self.angular_velocity += self.counter(self.angular_velocity) * 2 * math.pi
        elif delta_yaw < -math.pi: 
            self.angular_velocity -= self.counter(self.angular_velocity) * 2 * math.pi

        error_yaw = self.desired_yaw - current_yaw
        self.wz = self.kp * error_yaw

        if self.wz > self.Wz_max: 
            self.wz = self.Wz_max
        elif self.wz < -self.Wz_max:
            self.wz = -self.Wz_max

        self.cmd_vel_msg.angular.z = self.wz
        self.cmd_vel_publisher.publish(self.cmd_vel_msg)
        self.get_logger().info(f'Publishing: linear velocity: x = {self.cmd_vel_msg.linear.x}, y = {self.cmd_vel_msg.linear.y}, angular velocity: w = {self.cmd_vel_msg.angular.z}')

    def gamepad_callback(self, gamepad_msg):
        self.angular_velocity = (gamepad_msg.right_analog_x - 128)
        self.cmd_vel_msg.linear.x = (128 - gamepad_msg.left_analog_y)/128 * self.V_max
        self.cmd_vel_msg.linear.y = (gamepad_msg.left_analog_x - 128)/128 * self.V_max
          
def main():
    rclpy.init()
    robot_control = RobotControl()
    rclpy.spin(robot_control)
    robot_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()
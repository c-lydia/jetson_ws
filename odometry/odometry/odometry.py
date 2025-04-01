import rclpy 
from rclpy.node import Node 
from custom_messages.msg import EncoderFeedback
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math

class OdometryNode(Node): 
    def __init__(self): 
        super.__init__('odometry_node')
        self.encoder_msg_subscriber = self.create_subscription(EncoderFeedback, '/encoder_feedback', self.encoder_msg_callback, 10)
        self.odometry_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.imu_subscriber = self.create_subscription(Imu, 'imu/data_raw', self.imu_callback, 10)
        self.motor_position = [0.0, 0.0, 0.0, 0.0]
        self.motor_vel = [0.0, 0.0, 0.0, 0.0]
        self.dt = 0.01
        self.r = 0.06
        self.lx = 0.52
        self.ly = 0.52
        self.yaw_start = None 
        self.yaw_raw = None
        self.yaw = None 
        self.yaw_offset = 0.0 

    def imu_callback(self, imu_msg): 
        self.yaw_raw = imu_msg.angular_velocity.z

        if self.yaw_start == None: 
            self.yaw_start = self.yaw_raw

        self.yaw = self.yaw_raw - self.yaw_start + self.yaw_offset
        self.yaw_offset = self.yaw_raw

    def encoder_msg_callback(self, encoder_msg):
        motor_id = encoder_msg.can_id - 100

        for motor_id in range(4): 
            self.motor_vel[motor_id] = encoder_msg.speed
            self.motor_position[motor_id] = encoder_msg.position
            motor_id += 1

        linear_vel_x = (self.r * (self.motor_vel[0] + self.motor_vel[1] + self.motor_vel[2] + self.motor_vel[2]))/4
        linear_vel_y = (self.r * (-self.motor_vel[0] + self.motor_vel[1] - self.motor_vel[2] + self.motor_vel[3]))/4
        Wz = (2 * self.r * (-self.motor_vel[0] + self.motor_vel[1] + self.motor_vel[2] - self.motor_vel[3]))/4 * (self.lx + self.ly)

        Vx = math.cos(self.yaw) * linear_vel_x - math.sin(self.yaw) * linear_vel_y
        Vy = math.sin(self.yaw) * linear_vel_x + math.cos(self.yaw) * linear_vel_y

        x += Vx * self.dt 
        y += Vy * self.dt 
        z += Wz * self.dt

        odometry_msg = Odometry()

        odometry_msg.twist.twist.linear.x = Vx 
        odometry_msg.twist.twist.linear.y = Vy 
        odometry_msg.twist.twist.angular.z = Wz
        
        odometry_msg.pose.pose.position.x = x 
        odometry_msg.pose.pose.position.y = y 
        odometry_msg.pose.pose.position.z = z 

        self.odometry_publisher.publish(odometry_msg)

        print(f"Publishing: (velocity: Vx = {odometry_msg.twist.twist.linear.x:.2f}, Vy = {odometry_msg.twist.twist.linear.y:.2f}, Wz = {odometry_msg.twist.twist.angular.z:.2f}), (position: x = {odometry_msg.pose.pose.position.x:.2f}, y = {odometry_msg.pose.pose.position.y:.2f}, z = {odometry_msg.pose.pose.position.z:.2f}")

def main(): 
    rclpy.init()
    odometry = OdometryNode()
    rclpy.spin(odometry)
    odometry.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()
import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import Imu
from dc_gamepad_msgs.msg import GamePad
import math
import pynput 

class RobotControl(Node): 
    def __init__(self): 
        super().__init__('robot_control')
        self.gamepad_subscriber = self.create_subscription(GamePad, '/pad', self.gamepad_callback, 10)
        self.imu_subscriber = self.create_subscription(Imu, '/imu/data_raw', self.imu_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.yaw_start = None
        self.previous_yaw = None
        self.desired_yaw = 0.0
        self.joystick_left_x = 0.0
        self.joystick_left_y = 0.0
        self.joystick_right_x = 0.0
        self.current_yaw = 0.0 
        self.kp = 1.0 
        self.V_max = 1.0 
        self.Wz_max = 1.0 
        self.yaw_raw = 0.0 
        self.stopped = False 

        self.listener = pynput.keyboard.Listener(on_press = self.on_pressed)
        self.listener.start()

        self.timer = self.create_timer(0.01, self.timer_callback)

    def on_pressed(self, key): 
        try: 
            if key == pynput.keyboard.KeyCode.from_char('x'):
                self.stopped = True
        except AttributeError: 
            pass 

    def sign(self, x): 
        if x >= 0.0: 
            return 1.0 
        else: 
            return -1.0 

    def timer_callback(self): 
        Vx = self.joystick_left_x * self.V_max 

        if abs(Vx) > self.V_max: 
            Vx = self.sign(Vx) * self.V_max

        Vy = self.joystick_left_y * self.V_max 

        if abs(Vy) > self.V_max: 
            Vy = self.sign(Vy) * self.V_max

        self.desired_yaw += self.joystick_right_x * 0.01
        error_yaw = self.desired_yaw - self.current_yaw
        Wz = self.kp * error_yaw 

        if abs(Wz) > self.Wz_max: 
            Wz = self.sign(Wz) * self.Wz_max

        cmd_vel_msg = Twist()

        if self.stopped is True: 
            cmd_vel_msg.linear.x = 0.0 
            cmd_vel_msg.linear.y = 0.0 
            cmd_vel_msg.angular.z = 0.0
        else:
            cmd_vel_msg.linear.x = Vx
            cmd_vel_msg.linear.y = Vy
            cmd_vel_msg.angular.z = Wz

        self.cmd_vel_publisher.publish(cmd_vel_msg)
        self.get_logger().info(f'Publishing: Vx = {cmd_vel_msg.linear.x:.2f}, Vy = {cmd_vel_msg.linear.y:.2f}, Wz = {cmd_vel_msg.angular.z:.2f}')

    def imu_callback(self, imu_msg): 
        qx = imu_msg.orientation.x 
        qy = imu_msg.orientation.y 
        qz = imu_msg.orientation.z 
        qw = imu_msg.orientation.w 

        self.yaw_raw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy ** 2.0 + qz ** 2.0))

        if self.yaw_start == None: 
            self.yaw_start = self.yaw_raw
            self.previous_yaw = self.yaw_raw   
        
        difference_yaw = self.yaw_raw - self.previous_yaw

        if abs(difference_yaw) > math.pi: 
            if self.yaw_raw > 0.0: 
                self.overflow_counter -= 1.0
            else: 
                self.overflow_counter += 1.0

        self.current_yaw = self.yaw_raw - self.yaw_start + 2.0 * math.pi * self.overflow_counter

    def gamepad_callback(self, gamepad_msg): 
        self.joystick_left_x = (128.0 - gamepad_msg.left_analog_x) / 128.0
        self.joystick_left_y = (128.0 - gamepad_msg.left_analog_y) / 128.0
        self.joystick_right_x = (128.0 - gamepad_msg.right_analog_x) / 128.0

def main(): 
    rclpy.init()
    robot_control = RobotControl()
    rclpy.spin(robot_control)
    robot_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()
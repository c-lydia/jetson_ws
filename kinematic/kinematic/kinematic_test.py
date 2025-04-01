import rclpy
from rclpy.node import Node
from custom_messages.msg import MotorCommand 
from geometry_msgs.msg import Twist 

class kinematicPublisher(Node): 
    def __init__(self): 
        super().__init__('kinematic_publisher')
        self.cmd_vel_subscriber = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.kinematic_publisher = self.create_publisher(MotorCommand, '/publish_motor', 10)
        self.lx = 0.52
        self.ly = 0.52
        self.R = 0.06
    
    def cmd_vel_callback(self, cmd_vel_msg):
        Vx = cmd_vel_msg.linear.x 
        Vy = cmd_vel_msg.linear.y
        omega_z = cmd_vel_msg.angular.z
        V_wheel = [
            Vx/self.R - Vy/self.R - omega_z*(self.lx + self.ly)/(2.0*self.R),
            Vx/self.R + Vy/self.R + omega_z*(self.lx + self.ly)/(2.0*self.R),
            Vx/self.R - Vy/self.R + omega_z*(self.lx + self.ly)/(2.0*self.R), 
            Vx/self.R + Vy/self.R - omega_z*(self.lx + self.ly)/(2.0*self.R)
        ]
        V_speed = V_wheel
        for i in range(4):
            motor_msg = MotorCommand()
            motor_msg.speedmode = True
            motor_msg.can_id = i + 1
            motor_msg.goal = V_speed[i]
            self.kinematic_publisher.publish(motor_msg)
            self.get_logger().info(f'Publishing: speed = {V_speed}')

def main():
    rclpy.init()
    kinematic_publisher = kinematicPublisher()
    rclpy.spin(kinematic_publisher)
    kinematic_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()
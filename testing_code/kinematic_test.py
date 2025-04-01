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
        
        wheel_1 = Vx/self.R - Vy/self.R - omega_z*(self.lx + self.ly)/(2.0*self.R)
        wheel_2 = Vx/self.R + Vy/self.R + omega_z*(self.lx + self.ly)/(2.0*self.R)
        wheel_3 = Vx/self.R - Vy/self.R + omega_z*(self.lx + self.ly)/(2.0*self.R) 
        wheel_4 = Vx/self.R + Vy/self.R - omega_z*(self.lx + self.ly)/(2.0*self.R)

        motor_msg1 = MotorCommand()
        motor_msg1.speedmode = True
        motor_msg1.can_id = 1
        motor_msg1.goal = wheel_1
        self.kinematic_publisher.publish(motor_msg1)

        motor_msg2 = MotorCommand()
        motor_msg2.speedmode = True
        motor_msg2.can_id = 2
        motor_msg2.goal = wheel_2
        self.kinematic_publisher.publish(motor_msg2)

        motor_msg3 = MotorCommand()
        motor_msg3.speedmode = True
        motor_msg3.can_id = 3
        motor_msg3.goal = wheel_3
        self.kinematic_publisher.publish(motor_msg3)

        motor_msg4 = MotorCommand()
        motor_msg4.speedmode = True
        motor_msg4.can_id = 4
        motor_msg4.goal = wheel_4
        self.kinematic_publisher.publish(motor_msg4)
        self.get_logger().info(f'Publishing: speed = ({wheel_1:.2f}, {wheel_2:.2f}, {wheel_3:.2f}, {wheel_4:.2f})')

def main():
    rclpy.init()
    kinematic_publisher = kinematicPublisher()
    rclpy.spin(kinematic_publisher)
    kinematic_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()
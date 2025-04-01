import rclpy
from rclpy.node import Node
from custom_messages.msg import MotorCommand

class SwerveDriveNode(Node):
    def __init__(self):
        Node.__init__(self, 'Swerve_node')
        self.motor_publisher = self.create_publisher(MotorCommand, '/publish_motor', 10)
        self.my_timer = self.create_timer(0.01, self.send_motor)

    def send_motor(self):
        msg1 = MotorCommand()
        msg1.can_id = 1
        msg1.speedmode = True
        msg1.goal = 10.0

        msg2 = MotorCommand()
        msg2.can_id = 4
        msg2.speedmode = False
        msg2.goal = 0.0

        msg3 = MotorCommand()
        msg3.can_id = 6
        msg3.speedmode = False
        msg3.goal = 0.0

        self.motor_publisher.publish(msg1)
        # self.motor_publisher.publish(msg2)
        # self.motor_publisher.publish(msg3)


def main(args=None):
    rclpy.init(args=args)
    test_node = SwerveDriveNode()
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()






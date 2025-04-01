import rclpy 
from rclpy.node import Node 
from std_msgs.msg import UInt64

class PublisherNode1(Node):
    def __init__(self): 
        super().__init__('node_1')
        self.publisher_node_1 = self.create_publisher(UInt64, '/number', 10)
        self.node_1_timer = self.create_timer(1.0, self.node_1_timer_callback)
        self.i = 0

        self.subscriber_node_1 = self.create_subscription(UInt64, '/square', self.node_1_subscriber_callback, 10)

    def node_1_timer_callback(self): 
        msg = UInt64()
        self.i += 1
        msg.data = self.i
        self.publisher_node_1.publish(msg)
        print(f'Publishing: {msg.data}')

    def node_1_subscriber_callback(self, square_msg):
        print(f'Receiving: {square_msg.data}')

def main():
    rclpy.init()
    publisher_node_1 = PublisherNode1()
    rclpy.spin(publisher_node_1)
    publisher_node_1.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
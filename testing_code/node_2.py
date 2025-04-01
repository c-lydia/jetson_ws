import rclpy
from rclpy.node import Node 
from std_msgs.msg import UInt64

class SubscriberNode1(Node): 
    def __init__(self):
        super().__init__('subscriber_node_1')
        self.subscriber_node_1 = self.create_subscription(UInt64, '/number', self.node_1_callback, 10)
        self.publisher_node_2 = self.create_publisher(UInt64, '/square', 10)
        # self.num_receive = None
        self.node_2_timer = self.create_timer(1.0, self.node_2_timer_callback)

    def node_1_callback(self, msg):
        self.num_receive = msg.data
        #print(msg.data)
        
    def node_2_timer_callback(self):
            square_msg = UInt64()
            square_msg.data = self.num_receive ** 2
            print(square_msg.data)
            self.publisher_node_2.publish(square_msg)

def main(): 
    rclpy.init()
    subscriber_node_1 = SubscriberNode1()
    rclpy.spin(subscriber_node_1)
    subscriber_node_1.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
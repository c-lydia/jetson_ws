import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64

class Node1(Node): 
    def __init__(self):
        super().__init__('node_1')
        self.vector_publisher = self.create_publisher(Vector3, '/my_vector', 10)
        self.vector_timer = self.create_timer(1.0, self.vector_timer_callback)
        self.x = 1.0
        self.y = 2.0
        self.z = 3.0
        self.ditance_subscriber = self.create_subscription(Float64, '/distance', self.distance_subscriber_callback, 10)

    def vector_timer_callback(self): 
        msg = Vector3()
        msg.x = self.x 
        msg.y = self.y 
        msg.z = self.z 
        self.vector_publisher.publish(msg)
        self.get_logger().info(f'Publishing: vector = ({msg.x}, {msg.y}, {msg.z})')

    def distance_subscriber_callback(self, norm_msg): 
        self.get_logger().info(f'Receiveing: norm of vector = {norm_msg.data}')

def main(): 
    rclpy.init()
    node_1 = Node1()
    rclpy.spin(node_1)
    node_1.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
import math

class Node2(Node): 
    def __init__(self):
        super().__init__('node_2')
        self.vector_subscriber = self.create_subscription(Vector3, '/my_vector', self.vector_subscriber_callback, 10)
        self.distance_publisher = self.create_publisher(Float64, '/distance', 10)
        self.distance_timer = self.create_timer(1.0, self.distance_timer_callback)

    def vector_subscriber_callback(self, msg): 
        self.get_logger().info(f'Receiving: vector = ({msg.x}, {msg.y}, {msg.z})')
        self.x = msg.x
        self.y = msg.y 
        self.z = msg.z

    def distance_timer_callback(self): 
        norm_msg = Float64()
        norm_msg.data = math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)
        self.distance_publisher.publish(norm_msg)
        self.get_logger().info(f'Publishing: norm of vector = {norm_msg.data}')

def main(): 
    rclpy.init()
    node_2 = Node2()
    rclpy.spin(node_2)
    node_2.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()
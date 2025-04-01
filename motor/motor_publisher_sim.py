import rclpy
from rclpy.node import Node
from custom_messages.msg import EncoderFeedback

class EncoderPublisher(Node):
    def __init__(self):
        super().__init__('encoder_publisher')
        self.encoder_publisher = self.create_publisher(EncoderFeedback, '/encoder_feedback', 10)
        self.encoder_timer = self.create_timer(0.01, self.encoder_publisher_callback)
        self.speed = 1.0
        self.position = 0.0

    def encoder_publisher_callback(self):
        encoder_msg = EncoderFeedback()
        encoder_msg.can_id = 102
        encoder_msg.speed = self.speed
        encoder_msg.position = self.position
        self.encoder_publisher.publish(encoder_msg)
        self.get_logger().info(f'Publishing: encoder data = (can id = {encoder_msg.can_id}, speed = {encoder_msg.speed}, position = {encoder_msg.position})')
        self.position += self.speed * 0.01

def main(): 
    rclpy.init()
    encoder_publisher = EncoderPublisher()
    rclpy.spin(encoder_publisher)
    encoder_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()  
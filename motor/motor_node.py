import rclpy
from rclpy.node import Node
from custom_messages.msg import EncoderFeedback, MotorCommand 

class Motor(Node):
    def __init__(self):
        super().__init__('motor')
        self.encoder_feedback_sub = self.create_subscription(EncoderFeedback, '/encoder_feedback', self.encoder_feedback_callback, 10)
        self.motor_publisher = self.create_publisher(MotorCommand, '/publish_motor', 10)
        self.desired_position = 10.0
        self.kp = 1.0
        self.ki = 0.01
        self.kd = 0.001
        self.dt = 0.01 
        self.integral_error = 0.0
        # self.new_error = 0.0
        self.derivative_error = 0.0
        self.old_error = 0.0
        self.current_position = 0.0

    def encoder_feedback_callback(self, encoder_msg): 
        # self.current_position = 0.0
        # self.get_logger().info(f'data = (can id = {encoder_msg.can_id}, speed = {encoder_msg.speed}, position = {encoder_msg.position})')
        if encoder_msg.can_id == 103:
            self.current_position = encoder_msg.position
            error = self.desired_position - self.current_position
            self.integral_error += (error + self.old_error)/2 * self.dt 
            self.derivative_error += (error - self.old_error)/self.dt
            voltage =  self.kp * error  + self.integral_error * self.ki + self.derivative_error * self.kd
            motor_msg = MotorCommand()
            motor_msg.voltagemode = True
            motor_msg.can_id = 3
            motor_msg.goal = voltage 
            self.motor_publisher.publish(motor_msg)
            self.get_logger().info(f'Publishing: voltage = {motor_msg.goal}, position = {self.current_position}')
            # self.get_logger().info(f'Error: {self.old_error}, Integral Error: {self.integral_error}, Derivative Error: {self.derivative_error}, Voltage: {voltage}')
            self.old_error = error
        
def main(): 
    rclpy.init()
    encoder_feedback_sub = Motor()
    rclpy.spin(encoder_feedback_sub)
    encoder_feedback_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()  

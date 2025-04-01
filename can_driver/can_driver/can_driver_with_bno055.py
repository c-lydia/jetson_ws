#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
from custom_messages.msg import EncoderFeedback, MotorCommand, ServoCommand, PwmCommand, DigitalAndSolenoidCommand, DigitalAndAnalogFeedback
from geometry_msgs.msg import Quaternion
import struct
import subprocess
import array

class can_msg:
    def __init__(self, arbitration_id = 0, dlc = 8, data = [0, 0, 0, 0, 0, 0, 0, 0]):
        self.arbitration_id = arbitration_id
        self.dlc = dlc
        self.data = data
        self.serial_data = array.array('B', [0] * 13)
        self.serial_size = 13

    def convert_can_msg_to_serial(self, can_message):
        self.serial_data[0] = 0xFF
        self.serial_data[1] = can_message.arbitration_id >> 8
        self.serial_data[2] = can_message.arbitration_id & 0xFF
        self.serial_data[3] = can_message.dlc
        self.serial_data[4:12] = array.array('B', can_message.data)
        self.serial_data[12] = 0xFF

        return self.serial_data
    
    def convert_serial_to_can_msg(self, data):
        if data[0] == 0xFF and data[12] == 0xFF:
            self.arbitration_id = (data[1] << 8) | data[2]
            self.dlc = data[3]
            self.data = data[4:12]
            return True
        else:
            return False

class serial_buffer:
    def __init__(self, buffer_size):
        self.buffer_size = buffer_size
        self.data = array.array('B', [0] * buffer_size)
        self.head = 0
        self.tail = 0

    def push(self, data):
        length = len(data)
        head_index = self.head % self.buffer_size

        if head_index + length <= self.buffer_size:
            self.data[head_index:head_index + length] = array.array('B', data)
        else:
            first_part = self.buffer_size - head_index
            self.data[head_index:] = array.array('B', data[:first_part])
            self.data[:length - first_part] = array.array('B', data[first_part:])
        
        self.head += length

    def read(self, tail, length):
        tail_index = tail % self.buffer_size
     
        if tail_index + length <= self.buffer_size:
            return self.data[tail_index:tail_index + length]
        else:
            first_part = self.buffer_size - tail_index
            return self.data[tail_index:] + self.data[:length - first_part]

def run_subprocess(cmd):
    """Executes a command in the subprocess and returns the result."""
    return subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

class CanDriver(Node):
    def __init__(self, serial_port = "/dev/ttyUSB0"):
        Node.__init__(self, 'can_driver_node')  # Changed node name to MotorNode
        self.serial_port = serial_port
        self.setup_serial_interface()
        self.init_publisher()
        self.init_subscriber()
        self.canMsgData = [0, 0, 0, 0, 0, 0, 0, 0]
        
    def init_subscriber(self):
        self.motor_command_subscriber = self.create_subscription(MotorCommand, '/publish_motor', self.motor_command_callback, 10)
        self.servo_command_subscriber = self.create_subscription(ServoCommand, '/publish_servo', self.servo_command_callback, 10)
        self.pwm_command_subscriber = self.create_subscription(PwmCommand, '/publish_pwm', self.pwm_command_callback, 10)
        self.digital_solenoid_subscriber = self.create_subscription(DigitalAndSolenoidCommand, '/publish_digital_solenoid', self.digital_and_solenoid_command_callback, 10)
        
    def init_publisher(self):
        self.encoder_publisher = self.create_publisher(EncoderFeedback, '/encoder_feedback', 10)
        self.digital_and_analog_input_publisher = self.create_publisher(DigitalAndAnalogFeedback, '/digital_analog_feedback', 10)
        self.quaternion_publisher = self.create_publisher(Quaternion, '/quaternion', 10)

    def motor_command_callback(self, msg):
        self.canMsgData[0] = (
            msg.speedmode +
            (msg.stop << 1) +
            (msg.reset << 2) +
            (msg.voltagemode << 3)
        )
        goal_bytes = struct.pack("<f", msg.goal)
        self.canMsgData[2:6] = goal_bytes
        
        can_message = can_msg(arbitration_id=msg.can_id, data=self.canMsgData)
        self.send(can_message)
            
    def servo_command_callback(self, msg):
        servo_count = 4
        servo_value = [msg.servo1_value, msg.servo2_value, msg.servo3_value, msg.servo4_value]

        for i in range (servo_count):
            if servo_value[i] > 1.0:
                servo_value[i] = 1.0
            elif servo_value[i] < 0.0:
                servo_value[i] = 0.0

            servo_value_int = int(servo_value[i] * 16383.0)

            self.canMsgData[2 * i] = servo_value_int >> 8
            self.canMsgData[2 * i + 1] = servo_value_int & 0xFF

            if i == 0:
                self.canMsgData[0] = self.canMsgData[0] | 0xC0

        can_message = can_msg(arbitration_id=msg.can_id, data=self.canMsgData)
        self.send(can_message)

    def pwm_command_callback(self, msg):
        pwm_count = 4
        pwm_value = [msg.pwm1_value, msg.pwm2_value, msg.pwm3_value, msg.pwm4_value]

        for i in range (pwm_count):
            if pwm_value[i] > 1.0:
                pwm_value[i] = 1.0
            elif pwm_value[i] < 0.0:
                pwm_value[i] = 0.0

            pwm_value_int = int(pwm_value[i] * 16383.0)

            self.canMsgData[2 * i] = pwm_value_int >> 8
            self.canMsgData[2 * i + 1] = pwm_value_int & 0xFF

            if i == 0:
                self.canMsgData[0] = self.canMsgData[0] | 0x80

        can_message = can_msg(arbitration_id=msg.can_id, data=self.canMsgData)
        self.send(can_message)

    def digital_and_solenoid_command_callback(self, msg):
        digital_count = 4
        solenoid_count = 6

        digital_value = [msg.digital1_value, msg.digital2_value, msg.digital3_value, msg.digital4_value]
        solenoid_value = [msg.solenoid1_value, msg.solenoid2_value, msg.solenoid3_value, msg.solenoid4_value, msg.solenoid5_value, msg.solenoid6_value]

        self.canMsgData[0] = 0x40
        self.canMsgData[1] = 0
        self.canMsgData[2] = 0

        for i in range (digital_count):
            self.canMsgData[1] |= (digital_value[i] << i)
        
        for i in range(solenoid_count):
            self.canMsgData[2] |= (solenoid_value[i] << i)

        can_message = can_msg(arbitration_id=msg.can_id, data=self.canMsgData)
        self.send(can_message)
   
    def send(self, can_message):
        data = can_message.convert_can_msg_to_serial(can_message)
        self.transmit_serial_buffer.push(data)

    def setup_serial_interface(self):
        self.serial = serial.Serial(port=self.serial_port, baudrate=921600, rtscts=False, exclusive=True)
        self.receive_serial_buffer = serial_buffer(4096)
        self.transmit_serial_buffer = serial_buffer(4096)

        self.can_message_structure = can_msg()
        self.can_msg_received = can_msg()

        serial_time_step = 1.0 / 300
        self.serial_receive_timer = self.create_timer(serial_time_step, self.serial_receive_callback)
        self.serial_transmit_timer = self.create_timer(serial_time_step, self.serial_transmit_callback)

    def serial_transmit_callback(self):
        length = self.transmit_serial_buffer.head - self.transmit_serial_buffer.tail
        
        if length >= 13:
            data_to_transmit = self.transmit_serial_buffer.read(self.transmit_serial_buffer.tail, length)
            self.transmit_serial_buffer.tail = self.transmit_serial_buffer.head
        
            self.serial.write(data_to_transmit)

    def serial_receive_callback(self):     
        if self.serial.in_waiting:
            data = self.serial.read(self.serial.in_waiting)

            self.receive_serial_buffer.push(data)
            while self.receive_serial_buffer.head - self.receive_serial_buffer.tail >= self.can_msg_received.serial_size:
                data = self.receive_serial_buffer.read(self.receive_serial_buffer.tail, self.can_msg_received.serial_size)
                valid = self.can_msg_received.convert_serial_to_can_msg(data)
                if valid:
                    self.on_message_received(self.can_msg_received)
                    self.receive_serial_buffer.tail += self.can_msg_received.serial_size
                else:
                    self.receive_serial_buffer.tail += 1

    def on_message_received(self, msg):
        if 128 < msg.arbitration_id < 256:
            feedback_msg = EncoderFeedback()
            feedback_msg.can_id = msg.arbitration_id

            feedback_msg.position = struct.unpack_from("<f", msg.data, 0)[0]
            feedback_msg.speed = struct.unpack_from("<f", msg.data, 4)[0]
            self.encoder_publisher.publish(feedback_msg)

        elif 500 <= msg.arbitration_id <= 510:
            feedback_msg = DigitalAndAnalogFeedback()
            
            feedback_msg.can_id = msg.arbitration_id
            feedback_msg.digital1_value = (msg.data[0] & 1) == 1
            feedback_msg.digital2_value = ((msg.data[0] >> 1) & 1) == 1 
            feedback_msg.digital3_value = ((msg.data[0] >> 2) & 1) == 1
            feedback_msg.digital4_value = ((msg.data[0] >> 3) & 1) == 1

            feedback_msg.analog1_value = ((msg.data[1] << 4) + (msg.data[2] >> 4)) / 4095.0
            feedback_msg.analog2_value = (((msg.data[2] & 0x0F) << 8) + msg.data[3]) / 4095.0
            feedback_msg.analog3_value = ((msg.data[4] << 4) + (msg.data[5] >> 4)) / 4095.0
            feedback_msg.analog4_value = (((msg.data[5] & 0x0F) << 8) + msg.data[6]) / 4095.0

            self.digital_and_analog_input_publisher.publish(feedback_msg)

        if msg.arbitration_id == 256:

            sign_bit = msg.data[0] >> 7
            self.q = [0.0, 0.0, 0.0, 0.0]

            self.q[0] = (((msg.data[0] & 0x7F) << 8) + msg.data[1]) / 30000
            if sign_bit:
                self.q[0] *= -1.0
                
            sign_bit = msg.data[2] >> 7
            self.q[1] = (((msg.data[2] & 0x7F) << 8) + msg.data[3]) / 30000
            if sign_bit:
                self.q[1] *= -1.0

            sign_bit = msg.data[4] >> 7
            self.q[2] = (((msg.data[4] & 0x7F) << 8) + msg.data[5]) / 30000
            if sign_bit:
                self.q[2] *= -1.0

            sign_bit = msg.data[6] >> 7
            self.q[3] = (((msg.data[6] & 0x7F) << 8) + msg.data[7]) / 30000
            if sign_bit:
                self.q[3] *= -1.0

            quaternion_msg = Quaternion()
            quaternion_msg.w = self.q[0]
            quaternion_msg.x = self.q[1]
            quaternion_msg.y = self.q[2]
            quaternion_msg.z = self.q[3]
            
            self.quaternion_publisher.publish(quaternion_msg)
            

    def shutdown(self):
        self.serial_transmit_timer.cancel()
        self.serial_receive_timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    can_driver_node = CanDriver()
    try:
        rclpy.spin(can_driver_node)
    except KeyboardInterrupt:
        pass
    finally:
        can_driver_node.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

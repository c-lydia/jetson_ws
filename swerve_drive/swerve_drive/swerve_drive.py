import rclpy
from rclpy.node import Node
from dc_gamepad_msgs.msg import GamePad
from custom_messages.msg import MotorCommand, EncoderFeedback
from math import sin, cos, pi, sqrt, atan2
import time

class SwerveDriveNode(Node):
    def __init__(self):
        Node.__init__(self, 'Swerve_node')
        self.controller_listener = self.create_subscription(GamePad, '/pad', self.controller_callback, 10)
        self.encoder_listener = self.create_subscription(EncoderFeedback, '/encoder_feedback', self.encoder_callback, 10)
        self.motor_publisher = self.create_publisher(MotorCommand, '/publish_motor', 10)
        self.l = 0.24647 # idk yet update later
        self.r = 0.045
        self.desired_wheel_angle = [0.0, 0.0, 0.0]
        self.measured_wheel_angle = [0.0, 0.0, 0.0]
    
        self.command_wheel_angle = [0.0, 0.0, 0.0]
        self.command_wheel_speed = [0.0, 0.0, 0.0]

        self.low_speed_timer = time.perf_counter()
        self.no_activity = True

        self.penalty = False
        self.penalty_timer = time.perf_counter()

    def controller_callback(self, msg):
        max_Vx = 5.0
        max_Vy = 5.0
        max_Wz = 5.0

        Vx = max_Vx * float((msg.left_analog_x-128)/128)
        Vy = max_Vy * float(-(msg.left_analog_y-128)/128)
        Wz = max_Wz * float((msg.right_analog_x-128)/128)

        if abs(Vx) > 0.1 or abs(Vy) > 0.1 or abs(Wz) > 0.2:
            self.low_speed_timer = time.perf_counter()
        # print(f"{Vx:.2f}, {Vy:.2f}, {Wz:.2f}")

        self.inverse_kinematics(Vx, Vy, Wz)        

    def inverse_kinematics(self, Vx, Vy, Wz):

        l = self.l
        
        msgs = []

        for i in range(6):
            msg = MotorCommand()
            msg.can_id = (i+1)
            if i%2==0:
                msg.speedmode = True
            else:
                msg.speedmode = False
                msg.positionmode = True
            msgs.append(msg)

        V_i_x = [0, 0, 0]
        V_i_y = [0, 0, 0]

        target_wheel_speed = [0.0, 0.0, 0.0]
        target_wheel_angle = [0.0, 0.0, 0.0]

        nearest_wheel_angle = [0.0, 0.0, 0.0]

        #for the first_wheel
        V_i_x[0] = Wz * l + Vx
        V_i_y[0] = Vy

        #for the second wheel
        V_i_x[1] = -Wz * l * sin(pi / 6) + Vx
        V_i_y[1] = Wz * l * sin(pi / 3) + Vy

        #for the thrid wheel
        V_i_x[2] = -Wz * l * sin(pi / 6) + Vx
        V_i_y[2] = -Wz * l * sin(pi / 3) + Vy

        print(f"{Vx:.2f}, {Vy:.2f}, {Wz:.2f}")

        for i in range(3):
            target_wheel_speed[i] = sqrt(V_i_x[i]**2 + V_i_y[i]**2) / self.r #wheel speed is absolute
            target_wheel_angle[i] = atan2(V_i_x[i], V_i_y[i]) #wheel angle is absolute
            
            measured_wheel_angle_absolute = ((self.measured_wheel_angle[i] + pi) % (2 * pi)) - pi

            #two options: target_wheel_angle[i] or pi - target_wheel_angle[i]

            #for the first case
            theta1 = target_wheel_angle[i]
            theta2 = target_wheel_angle[i] - pi

            error1 = ((theta1 - self.measured_wheel_angle[i] + pi) % (2 * pi)) - pi
            error2 = ((theta2 - self.measured_wheel_angle[i] + pi) % (2 * pi)) - pi

            # print(theta1, theta2)

            if abs(error1) <= pi / 2:
                self.command_wheel_angle[i] = self.measured_wheel_angle[i] + error1
                self.command_wheel_speed[i] = target_wheel_speed[i]
            else:
                self.command_wheel_angle[i] = self.measured_wheel_angle[i] + error2
                self.command_wheel_speed[i] = -target_wheel_speed[i]

            elapsed_time = time.perf_counter() - self.low_speed_timer

            if elapsed_time > 0.7:
                self.no_activity = True
            else:
                self.no_activity = False

            if not self.penalty:
                if abs(self.measured_wheel_angle[i]) > 2 * 2 * pi:
                    self.penalty = True
                    self.penalty_timer = time.perf_counter()
            else:
                elapsed_time = time.perf_counter() - self.penalty_timer

                if elapsed_time > 1.0:
                    self.penalty = False
            
            if self.no_activity or self.penalty:
                self.command_wheel_angle[i] = 0.0

            msgs[i*2].goal = self.command_wheel_speed[i]
            msgs[i*2+1].goal = self.command_wheel_angle[i]

        #print(f"{msgs[0].goal:.2f} {msgs[2].goal:.2f} {msgs[4].goal:.2f}                 {msgs[1].goal:.2f} {msgs[3].goal:.2f} {msgs[5].goal:.2f}       {self.measured_wheel_angle[0]} {self.measured_wheel_angle[1]} {self.measured_wheel_angle[2]}")
        
        for i in range(6):
            self.motor_publisher.publish(msgs[i])

        # for i in range(3):
        #     self.motor_publisher.publish(msgs[i*2])

    def encoder_callback(self, msg):
        if msg.can_id==102:
            self.measured_wheel_angle[0] = msg.position
        elif msg.can_id == 104:
            self.measured_wheel_angle[1] = msg.position
        elif msg.can_id == 106:
            self.measured_wheel_angle[2] = msg.position

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



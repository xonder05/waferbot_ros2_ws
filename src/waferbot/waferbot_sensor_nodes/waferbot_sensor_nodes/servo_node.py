import rclpy
from rclpy.node import Node

from interfaces.action import Servo
from std_msgs.msg import Float32

import Adafruit_PCA9685
import time

class ServoNode(Node):
    def __init__(self):
        super().__init__("servo_node")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('pwm_gen_channel', rclpy.Parameter.Type.INTEGER),
                ('default_position', rclpy.Parameter.Type.INTEGER),
                ('top_rotation_limit', rclpy.Parameter.Type.INTEGER),
                ('bottom_rotation_limit', rclpy.Parameter.Type.INTEGER),
            ]
        )
        self.pwm_gen_channel = self.get_parameter('pwm_gen_channel').get_parameter_value().integer_value
        self.default_position = self.get_parameter('default_position').get_parameter_value().integer_value
        self.top_rotation_limit = self.get_parameter('top_rotation_limit').get_parameter_value().integer_value
        self.bottom_rotation_limit = self.get_parameter('bottom_rotation_limit').get_parameter_value().integer_value
        
        self.angle_command = self.create_subscription(Float32, "/servo_pos", self.callback, 10)

        self.servo = Adafruit_PCA9685.PCA9685(busnum=1)
        self.servo.set_pwm_freq(50)
        self.servo.set_pwm(self.pwm_gen_channel, 0, self.default_position)

        self.get_logger().info('InitDone')

    def __del__(self):
        self.servo.set_pwm(self.pwm_gen_channel, 0, self.default_position)

    def callback(self, msg):

        if msg.data >= 0:
            pwm = self.default_position + (self.top_rotation_limit - self.default_position) * msg.data / 45
        else:
            pwm = self.default_position + (self.default_position - self.bottom_rotation_limit) * msg.data / 45

        pos = self.checkLimits(int(pwm))
        self.servo.set_pwm(self.pwm_gen_channel, 0, pos)


    #limit the servo rotation angles so the camera does not break itself
    def checkLimits(self, target_position):
        if target_position < self.bottom_rotation_limit:
            return self.bottom_rotation_limit
        elif target_position > self.top_rotation_limit:
            return self.top_rotation_limit
        else:
            return target_position

def main():
    rclpy.init()
    node = ServoNode()
    rclpy.spin(node)
    rclpy.shutdown()
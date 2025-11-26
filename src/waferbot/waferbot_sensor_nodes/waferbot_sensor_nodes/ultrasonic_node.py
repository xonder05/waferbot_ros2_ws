import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from sensor_msgs.msg import Range

import pigpio

class UltrasonicNode(Node):
    def __init__(self):
        super().__init__("ultrasonic_node")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('tr_pin', rclpy.Parameter.Type.INTEGER),
                ('ec_pin', rclpy.Parameter.Type.INTEGER),
            ]
        )
        self.tr_pin = self.get_parameter('tr_pin').get_parameter_value().integer_value
        self.ec_pin = self.get_parameter('ec_pin').get_parameter_value().integer_value
        
        self.range_publisher = self.create_publisher(Range, "range", 10)
        self.timer = self.create_timer(0.1, self.checkdist)

        self.gpio = pigpio.pi()
        
        if not self.gpio.connected:
            raise RuntimeError("Could not connect to local pigpiod deamon")

        self.gpio.set_mode(self.tr_pin, pigpio.OUTPUT)
        self.gpio.set_mode(self.ec_pin, pigpio.INPUT)

        self.previous_distance = 0

        self.t1 = 0
        self.t2 = 0

        self.get_logger().info("InitDone")


    def echo_callback(self, gpio, level, tick):

        if (level == 1) and (self.t1 == 0) and (self.t2 == 0):
            self.t1 = tick

        elif (level == 0) and (self.t1 != 0) and (self.t2 == 0):
            self.t2 = tick
            self.ecb.cancel()

        else:
            self.t1 = -1
            self.t2 = -1
            self.ecb.cancel()

    def checkdist(self):

        # got valid measurement that needs to be handeled
        if (self.t1 > 0) and (self.t2 > 0):

            duration = (self.t2 - self.t1) / 1_000_000.0

            #calculate distance from captured time stamps and speed of sound in air
            distance = round(duration * 340 / 2,2)

            #publish distance
            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "ultrasonic"
            msg.radiation_type = 0 # [ULTRASOUND]
            msg.field_of_view = 0.26179939 # [rad] (15°)
            msg.min_range = 0.020 # [m]
            msg.max_range = 4.0 # [m]
            msg.range = distance

            self.range_publisher.publish(msg)

        # reset
        self.t1 = 0
        self.t2 = 0

        # setup callback
        self.ecb = self.gpio.callback(self.ec_pin, pigpio.EITHER_EDGE, self.echo_callback)

        # order sensor to start measurement
        self.gpio.write(self.tr_pin, 0)
        time.sleep(0.000015)
        self.gpio.write(self.tr_pin, 1)
        time.sleep(0.000015)
        self.gpio.write(self.tr_pin, 0)
        

def main():
    rclpy.init()
    node = UltrasonicNode()
    rclpy.spin(node)
    rclpy.shutdown()

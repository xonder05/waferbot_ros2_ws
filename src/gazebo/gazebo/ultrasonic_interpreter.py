import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range

class UltrasonicInterpreter(Node):

    def __init__(self):
        super().__init__("ultrasonic_interpreter")
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('gazebo_topic', rclpy.Parameter.Type.STRING),
                ('ros_topic', rclpy.Parameter.Type.STRING),
            ]
        )
        self.gazebo_topic = self.get_parameter('gazebo_topic').get_parameter_value().string_value
        self.ros_topic = self.get_parameter('ros_topic').get_parameter_value().string_value

        self.subscriber = self.create_subscription(LaserScan, self.gazebo_topic, self.callback, 10)
        self.range_publisher = self.create_publisher(Range, self.ros_topic, 10)

        self.get_logger().info("InitDone")

    def callback(self, msg: LaserScan):
        
        # gazebo uses lidar with three rays instead of ultrasonic, so this needs to convert it to ultrasonic like data
        left = msg.ranges[0]
        middle = msg.ranges[1]
        right = msg.ranges[2]
        distance = min(left, min(middle, right))

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "ultrasonic"
        msg.radiation_type = 0 # [ULTRASOUND]
        msg.field_of_view = 0.26179939 # [rad] (15°)
        msg.min_range = 0.020 # [m]
        msg.max_range = 4.0 # [m]
        msg.range = distance

        self.range_publisher.publish(msg)

def main():
    rclpy.init()
    node = UltrasonicInterpreter()
    rclpy.spin(node)
    rclpy.shutdown()
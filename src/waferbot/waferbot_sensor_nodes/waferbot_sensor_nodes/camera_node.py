import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image

from picamera2.picamera2 import Picamera2
from picamera2.encoders import H264Encoder

import cv2
import numpy as np

class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('video_capture', rclpy.Parameter.Type.STRING),
                ('video_resolution_width', rclpy.Parameter.Type.INTEGER),
                ('video_resolution_height', rclpy.Parameter.Type.INTEGER),
                ('video_framerate', rclpy.Parameter.Type.DOUBLE),
            ]
        )
        self.video_capture = self.get_parameter('video_capture').get_parameter_value().string_value
        self.video_resolution_width = self.get_parameter('video_resolution_width').get_parameter_value().integer_value
        self.video_resolution_height = self.get_parameter('video_resolution_height').get_parameter_value().integer_value
        self.video_framerate = self.get_parameter('video_framerate').get_parameter_value().double_value
        
        self.publisher = self.create_publisher(CompressedImage, "/camera_stream", 10)
        self.timer = self.create_timer(self.video_framerate, self.get_frame)
        
        self.picam2 = Picamera2()

        config = self.picam2.create_video_configuration(
            main={
                "size": (self.video_resolution_width, self.video_resolution_height),
                "format": "RGB888"
            }
        )

        self.picam2.configure(config)
        self.picam2.start()

        self.get_logger().info("InitDone")

    #captures image, compresses it and sends to topic
    def get_frame(self):

        frame = self.picam2.capture_array()
        
        frame_rotated = cv2.rotate(frame, cv2.ROTATE_180)

        ros_image = CompressedImage()
        ros_image.format = "jpeg"
        ros_image.data = cv2.imencode('.jpg', frame_rotated)[1].tobytes()
        self.publisher.publish(ros_image)
            
def main():
    rclpy.init()
    node = CameraNode()
    rclpy.spin(node)
    node.picam2.stop()
    rclpy.shutdown()

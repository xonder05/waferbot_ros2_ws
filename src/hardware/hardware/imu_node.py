import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu

import math
import transforms3d

from mpu6050 import mpu6050

class ImuNode(Node):
    def __init__(self):
        super().__init__("imu_node")
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('acc_x_offset', rclpy.Parameter.Type.DOUBLE),
                ('acc_y_offset', rclpy.Parameter.Type.DOUBLE),
                ('acc_z_offset', rclpy.Parameter.Type.DOUBLE),
                ('gyro_x_offset', rclpy.Parameter.Type.DOUBLE),
                ('gyro_y_offset', rclpy.Parameter.Type.DOUBLE),
                ('gyro_z_offset', rclpy.Parameter.Type.DOUBLE),
                ('acc_x_low_cutoff', rclpy.Parameter.Type.DOUBLE),
                ('acc_y_low_cutoff', rclpy.Parameter.Type.DOUBLE),
                ('acc_z_low_cutoff', rclpy.Parameter.Type.DOUBLE),
                ('gyro_x_low_cutoff', rclpy.Parameter.Type.DOUBLE),
                ('gyro_y_low_cutoff', rclpy.Parameter.Type.DOUBLE),
                ('gyro_z_low_cutoff', rclpy.Parameter.Type.DOUBLE),
            ]
        )
        self.acc_x_offset = self.get_parameter('acc_x_offset').get_parameter_value().double_value
        self.acc_y_offset = self.get_parameter('acc_y_offset').get_parameter_value().double_value
        self.acc_z_offset = self.get_parameter('acc_z_offset').get_parameter_value().double_value
        self.gyro_x_offset = self.get_parameter('gyro_x_offset').get_parameter_value().double_value
        self.gyro_y_offset = self.get_parameter('gyro_y_offset').get_parameter_value().double_value
        self.gyro_z_offset = self.get_parameter('gyro_z_offset').get_parameter_value().double_value
        self.acc_x_low_cutoff = self.get_parameter('acc_x_low_cutoff').get_parameter_value().double_value
        self.acc_y_low_cutoff = self.get_parameter('acc_y_low_cutoff').get_parameter_value().double_value
        self.acc_z_low_cutoff = self.get_parameter('acc_z_low_cutoff').get_parameter_value().double_value
        self.gyro_x_low_cutoff = self.get_parameter('gyro_x_low_cutoff').get_parameter_value().double_value
        self.gyro_y_low_cutoff = self.get_parameter('gyro_y_low_cutoff').get_parameter_value().double_value
        self.gyro_z_low_cutoff = self.get_parameter('gyro_z_low_cutoff').get_parameter_value().double_value
        
        self.publisher = self.create_publisher(Imu, "/imu", 10)
        self.tf_timer = self.create_timer(0.05, self.publish_measurement)
        
        self.sensor = sensor = mpu6050(0x68)
        sensor.set_filter_range(0x05)

        self.prev_time = self.get_clock().now()
        self.prev_gyro_data = self.sensor.get_gyro_data()
        self.orientation = {'x': 0.0, 'y': 0.0, 'z': 0.0}

        self.get_logger().info("InitDone")

    def get_accel_data(self):
        accel_data = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        #average value from x samples (not used in final solution but the option is there)
        for i in range(0, 1):
            new_data = self.sensor.get_accel_data()

            accel_data['x'] += (new_data['x'] + self.acc_x_offset)
            accel_data['y'] += (new_data['y'] + self.acc_y_offset)
            accel_data['z'] += (new_data['z'] + self.acc_z_offset)

        accel_data['x'] = accel_data['x'] / 1.0
        accel_data['y'] = accel_data['y'] / 1.0
        accel_data['z'] = accel_data['z'] / 1.0

        #cut low values (so the robot does not drift while stationary)
        if abs(accel_data['x']) < self.acc_x_low_cutoff:
            accel_data['x'] = 0

        if abs(accel_data['y']) < self.acc_y_low_cutoff:
            accel_data['y'] = 0

        if abs(accel_data['z']) < self.acc_z_low_cutoff:
            accel_data['z'] = 0

        #align physical axis with those used by ros2
        accel_data['x'] *= -1

        tmp = accel_data['y']
        accel_data['y'] = accel_data['z']
        accel_data['z'] = tmp

        return accel_data

    def get_gyro_data(self):
        gyro_data_deg = {'x': 0.0, 'y': 0.0, 'z': 0.0}

        #average value from x samples (not used in final solution but the option is there)
        for i in range(0, 1):
            new_data = self.sensor.get_gyro_data()

            gyro_data_deg['x'] += (new_data['x'] + self.gyro_x_offset)
            gyro_data_deg['y'] += (new_data['y'] + self.gyro_y_offset)
            gyro_data_deg['z'] += (new_data['z'] + self.gyro_z_offset)

        gyro_data_deg['x'] = gyro_data_deg['x'] / 1.0
        gyro_data_deg['y'] = gyro_data_deg['y'] / 1.0
        gyro_data_deg['z'] = gyro_data_deg['z'] / 1.0

        #cut low values (so the robot does not drift while stationary)
        if abs(gyro_data_deg['x']) < self.gyro_x_low_cutoff:
            gyro_data_deg['x'] = 0

        if abs(gyro_data_deg['y']) < self.gyro_y_low_cutoff:
            gyro_data_deg['y'] = 0
        
        if abs(gyro_data_deg['z']) < self.gyro_z_low_cutoff:
            gyro_data_deg['z'] = 0

        #align physical axis with those used by ros2
        tmp = gyro_data_deg['y']
        gyro_data_deg['y'] = gyro_data_deg['z']
        gyro_data_deg['z'] = tmp

        gyro_data_deg['z'] *= -1

        #convertion from degrees to radians
        gyro_data = {
            'x': math.radians(gyro_data_deg['x']),
            'y': math.radians(gyro_data_deg['y']),
            'z': math.radians(gyro_data_deg['z'])
        }

        return gyro_data

    def publish_measurement(self):
        #get information for calcualtions
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds * 1e-9
        
        gyro_data = self.get_gyro_data()
        accel_data = self.get_accel_data()

        for axis in self.orientation.keys():
            self.orientation[axis] += (gyro_data[axis] + self.prev_gyro_data[axis]) / 2 * dt

        # todo covariance
        msg = Imu()
        msg.header.frame_id = ""
        msg.header.stamp = self.get_clock().now().to_msg()

        # orientation
        quaternion = transforms3d.euler.euler2quat(self.orientation['x'], 
                                                   self.orientation['y'], 
                                                   self.orientation['z'])
        msg.orientation.w = quaternion[0]
        msg.orientation.x = quaternion[1]
        msg.orientation.y = quaternion[2]
        msg.orientation.z = quaternion[3]

        # angular
        msg.angular_velocity.x = float(gyro_data['x'])
        msg.angular_velocity.y = float(gyro_data['y'])
        msg.angular_velocity.z = float(gyro_data['z'])
        # msg.angular_velocity_covariance

        # linear
        msg.linear_acceleration.x = float(accel_data['x'])
        msg.linear_acceleration.y = float(accel_data['y'])
        msg.linear_acceleration.z = float(accel_data['z'])
        # msg.linear_acceleration_covariance

        self.publisher.publish(msg)

        self.prev_time = current_time
        self.prev_gyro_data = gyro_data
        

def main():
    rclpy.init()
    node = ImuNode()
    rclpy.spin(node)
    rclpy.shutdown()

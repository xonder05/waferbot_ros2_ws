import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

import tf2_ros
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import TwistStamped
from waferbot_msgs.action import Spin
from waferbot_msgs.action import Drive

import transforms3d


class MotionExecutorNode(Node):
    def __init__(self):
        super().__init__("motion_executor_node")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('update_rate', rclpy.Parameter.Type.INTEGER),
                
                ('angular_goal_accuracy', rclpy.Parameter.Type.DOUBLE),
                ('angular_decel_distance', rclpy.Parameter.Type.DOUBLE),

                ('min_angular_speed', rclpy.Parameter.Type.DOUBLE),
                ('max_angular_speed', rclpy.Parameter.Type.DOUBLE),
                ('max_angular_accel', rclpy.Parameter.Type.DOUBLE),

                ('linear_goal_accuracy', rclpy.Parameter.Type.DOUBLE),
                ('linear_decel_distance', rclpy.Parameter.Type.DOUBLE),

                ('min_linear_speed', rclpy.Parameter.Type.DOUBLE),
                ('max_linear_speed', rclpy.Parameter.Type.DOUBLE),
                ('max_linear_accel', rclpy.Parameter.Type.DOUBLE),
            ]
        )
        self.update_rate = self.get_parameter("update_rate").get_parameter_value().integer_value

        self.angular_goal_accuracy = self.get_parameter("angular_goal_accuracy").get_parameter_value().double_value
        self.angular_decel_distance = self.get_parameter("angular_decel_distance").get_parameter_value().double_value

        self.min_angular_speed = self.get_parameter("min_angular_speed").get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter("max_angular_speed").get_parameter_value().double_value
        self.max_angular_accel = self.get_parameter("max_angular_accel").get_parameter_value().double_value

        self.linear_goal_accuracy = self.get_parameter("linear_goal_accuracy").get_parameter_value().double_value
        self.linear_decel_distance = self.get_parameter("linear_decel_distance").get_parameter_value().double_value

        self.min_linear_speed = self.get_parameter("min_linear_speed").get_parameter_value().double_value
        self.max_linear_speed = self.get_parameter("max_linear_speed").get_parameter_value().double_value
        self.max_linear_accel = self.get_parameter("max_linear_accel").get_parameter_value().double_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.spin_action = ActionServer(self, Spin, "spin",
                                        goal_callback=self.spin_goal_callback,
                                        handle_accepted_callback=self.spin_handle_accepted_callback,
                                        cancel_callback=self.spin_cancel_callback,
                                        execute_callback=self.spin_execute_callback)
                                        
        self.drive_action = ActionServer(self, Drive, "drive", 
                                         goal_callback=self.drive_goal_callback,
                                         handle_accepted_callback=self.drive_handle_accepted_callback,
                                         cancel_callback=self.drive_cancel_callback,
                                         execute_callback=self.drive_execute_callback)

        self.velocity_publisher = self.create_publisher(TwistStamped, "diff_drive_controller/cmd_vel", 10)

        self.sleep_duration = 1 / self.update_rate

        self.goal_handle = None
        self.get_logger().info("InitDone")

#--------------------------------------- Spin ---------------------------------------

    def get_current_rotation(self):
        try:
            msg = self.tf_buffer.lookup_transform("waferbot/odom", "waferbot/base_link", tf2_ros.Time())
            quaternion = msg.transform.rotation
            roll, pitch, yaw = transforms3d.euler.quat2euler([quaternion.w, quaternion.x, quaternion.y, quaternion.z])
            return roll, pitch, yaw

        except TransformException as ex:
            self.get_logger().info(f'Could not transform: {ex}')
            return 0.0, 0.0, 0.0

    def normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def spin_goal_callback(self, goal_request):

        if self.goal_handle is not None and self.goal_handle.is_active:
            self.get_logger().info(f"Rejecting goal request since previous one is still being executed")
            return GoalResponse.REJECT
        else:
            return GoalResponse.ACCEPT
    
    def spin_handle_accepted_callback(self, goal_handle):

        self.goal_handle = goal_handle
        self.timer = self.create_timer(self.sleep_duration, self.spin_callback)

        _, _, yaw = self.get_current_rotation()
        self.goal_position = self.normalize_angle(yaw + goal_handle.request.angle)

        self.speed = 0.0
        self.target_reached = False

    def spin_callback(self):

        _, _, yaw = self.get_current_rotation()
        remaining = self.normalize_angle(self.goal_position - yaw)

        if abs(remaining) <= self.angular_goal_accuracy:

            if not self.target_reached:

                self.target_reached = True
                self.motor_response_countdown = 10

                msg = TwistStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.twist.angular.z = 0.0
                self.velocity_publisher.publish(msg)

            else:
                if self.motor_response_countdown > 0:
                    self.motor_response_countdown -= 1

                else:
                    self.timer.cancel()
                    self.goal_handle.execute()

            return
        
        else:
            self.target_reached = False


        # speedup and hold
        if abs(remaining) > self.angular_decel_distance:
            self.speed = min(self.max_angular_speed, self.speed + self.max_angular_accel * self.sleep_duration)

        # slowdown
        else:
            self.speed = max(self.min_angular_speed, (remaining / self.angular_decel_distance) * self.max_angular_speed)

        # speed = self.speed if self.goal_handle.request.angle > 0 else -self.speed
        
        speed = self.speed if remaining > 0 else -self.speed

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.angular.z = speed
        self.velocity_publisher.publish(msg)

        feedback_msg = Spin.Feedback()
        feedback_msg.remaining = remaining
        self.goal_handle.publish_feedback(feedback_msg)

    def spin_cancel_callback(self, goal_handle):
        self.timer.cancel()
        goal_handle.execute()
        return CancelResponse.ACCEPT

    def spin_execute_callback(self, goal_handle):

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.z = 0.0
        self.velocity_publisher.publish(msg)
        
        if goal_handle.is_cancel_requested:
            goal_handle.abort()
        else:
            goal_handle.succeed()
        
        return Spin.Result()

#--------------------------------------- Drive ---------------------------------------

    def get_current_position(self):
        try:
            msg = self.tf_buffer.lookup_transform("waferbot/odom", "waferbot/base_link", tf2_ros.Time())
            position = msg.transform.translation
            return position.x, position.y, position.z

        except TransformException as ex:
            self.get_logger().info(f"Transform exception: {ex}")
            return 0.0, 0.0, 0.0
    

    def drive_goal_callback(self, goal_request):
        if self.goal_handle is not None and self.goal_handle.is_active:
            self.get_logger().info(f"Rejecting goal request since previous one is still being executed")
            return GoalResponse.REJECT
        else:
            return GoalResponse.ACCEPT


    def drive_handle_accepted_callback(self, goal_handle):
        self.goal_handle = goal_handle
        self.timer = self.create_timer(self.sleep_duration, self.drive_callback)

        self.x0, self.y0, _ = self.get_current_position()
        self.goal_distance = goal_handle.request.distance

        self.speed = 0.0
        self.target_reached = False

    def drive_callback(self):

        x, y, _ = self.get_current_position()
        r = math.hypot(x - self.x0, y - self.y0)
        remaining = abs(self.goal_distance) - r

        if abs(remaining) <= self.linear_goal_accuracy:

            if not self.target_reached:

                self.target_reached = True
                self.motor_response_countdown = 10

                msg = TwistStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.twist.linear.x = 0.0
                self.velocity_publisher.publish(msg)

            else:
                if self.motor_response_countdown > 0:
                    self.motor_response_countdown -= 1

                else:
                    self.timer.cancel()
                    self.goal_handle.execute()

            return
        
        else:
            self.target_reached = False


        # speedup and hold
        if abs(remaining) > self.linear_decel_distance:
            self.speed = min(self.max_linear_speed, self.speed + self.max_linear_accel * self.sleep_duration)

        #slowdown
        else:
            self.speed = max(self.min_linear_speed, (remaining / self.linear_decel_distance) * self.max_linear_speed)

        # if the command is forward or backward
        speed = self.speed if self.goal_distance > 0 else -self.speed

        # reverse direction if overshoot
        speed = speed if remaining > 0 else -speed
        
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = speed
        self.velocity_publisher.publish(msg)

        feedback_msg = Drive.Feedback()
        feedback_msg.remaining = remaining
        self.goal_handle.publish_feedback(feedback_msg)

    def drive_cancel_callback(self, goal_handle):
        self.timer.cancel()
        goal_handle.execute()
        return CancelResponse.ACCEPT

    def drive_execute_callback(self, goal_handle):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = 0.0
        self.velocity_publisher.publish(msg)
        
        if goal_handle.is_cancel_requested:
            goal_handle.abort()
        else:
            goal_handle.succeed()
        
        return Drive.Result()


def main():
    rclpy.init()
    node = MotionExecutorNode()
    rclpy.spin(node)
    rclpy.shutdown()

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Float64MultiArray
from waferbot_msgs.action import Spin, Drive
from sensor_msgs.msg import Range

import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

class WanderingNode(Node):
    def __init__(self):
        super().__init__("wandering_node")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('update_rate', rclpy.Parameter.Type.INTEGER),
                
                ('angular_goal_accuracy', rclpy.Parameter.Type.DOUBLE),
                ('angular_decel_distance', rclpy.Parameter.Type.DOUBLE),
            ]
        )
        self.update_rate = self.get_parameter("update_rate").get_parameter_value().integer_value
        self.angular_goal_accuracy = self.get_parameter("angular_goal_accuracy").get_parameter_value().double_value
        self.angular_decel_distance = self.get_parameter("angular_decel_distance").get_parameter_value().double_value


        self.range_subscriber = self.create_subscription(Range, "range", self.range_callback, 10)
        self.servo_publisher = self.create_publisher(Float64MultiArray, "forward_command_controller/commands", 10)
        self.spin_action_client = ActionClient(self, Spin, "spin")
        self.drive_action_client = ActionClient(self, Drive, "drive")

        # todo wait for action and service servers to be online
        self.timer = self.create_timer(1, self.callback)
        self.next_state = "Scan"

        self.scan_angles = [-0.7853, -0.3927, 0, 0.3927, 0.7853]
        self.scan_results = []
        self.scan_index = 0
        self.scan_save_distance = False

        self.fuzzy_init()

        self.get_logger().info("InitDone")

    def range_callback(self, msg):
        
        if self.scan_save_distance == True:

            self.scan_save_distance = False
            self.get_logger().info(f"saving distance {msg.range}")

            self.scan_results.append(msg.range)
            self.scan()

    def scan(self):

        if self.scan_index < len(self.scan_angles):

            angle = self.scan_angles[self.scan_index]

            msg = Float64MultiArray()
            msg.data = [angle]
            self.servo_publisher.publish(msg)

            time.sleep(1.0)

            self.scan_index += 1
            self.scan_save_distance = True

        else:
            msg = Float64MultiArray()
            msg.data = [0]
            self.servo_publisher.publish(msg)

            self.callback()


    def callback(self):
        
        self.get_logger().info(f"callback {self.next_state}")
        self.timer.cancel()


        if self.next_state == "Scan":
            self.scan_results = []
            self.scan_index = 0
            self.scan()
            self.next_state = "Fuzzy"
            return

        if self.next_state == "Fuzzy":
            self.get_logger().info(f"{self.scan_results}")

            fuzzy_result = self.fuzzy_compute(self.scan_results)

            self.get_logger().info(f"fuzzy: {fuzzy_result}")

            if abs(fuzzy_result["spin"]) > 0.001:
                self.send_spin_goal(fuzzy_result["spin"])

            elif abs(fuzzy_result["drive"]) > 0.001:
                self.send_drive_goal(fuzzy_result["drive"])

            else:
                self.get_logger().info("well, fuzzy gave up")

            self.next_state = "Scan"
            return


    def fuzzy_init(self):

        warn_range = [0, 0, 0.25, 0.25]
        close_range = [0.25, 0.5, 1.0]
        medium_range = [0.5, 1.0, 1.5]
        far_range = [1.0, 2.0, 10, 10]

        left = ctrl.Antecedent(np.arange(0, 10, 0.01), "left")
        left["warn"] = fuzz.trapmf(left.universe, warn_range)
        left["close"] = fuzz.trimf(left.universe, close_range)
        left["medium"] = fuzz.trimf(left.universe, medium_range)
        left["far"] = fuzz.trapmf(left.universe, far_range)

        slight_left = ctrl.Antecedent(np.arange(0, 10, 0.01), "slight_left")
        slight_left["warn"] = fuzz.trapmf(slight_left.universe, warn_range)
        slight_left["close"] = fuzz.trimf(slight_left.universe, close_range)
        slight_left["medium"] = fuzz.trimf(slight_left.universe, medium_range)
        slight_left["far"] = fuzz.trapmf(slight_left.universe, far_range)

        front = ctrl.Antecedent(np.arange(0, 10, 0.01), "front")
        front["warn"] = fuzz.trapmf(front.universe, warn_range)
        front["close"] = fuzz.trimf(front.universe, close_range)
        front["medium"] = fuzz.trimf(front.universe, medium_range)
        front["far"] = fuzz.trapmf(front.universe, far_range)

        slight_right  = ctrl.Antecedent(np.arange(0, 10, 0.01), "slight_right")
        slight_right["warn"] = fuzz.trapmf(slight_right.universe, warn_range)
        slight_right["close"] = fuzz.trimf(slight_right.universe, close_range)
        slight_right["medium"] = fuzz.trimf(slight_right.universe, medium_range)
        slight_right["far"] = fuzz.trapmf(slight_right.universe, far_range)

        right = ctrl.Antecedent(np.arange(0, 10, 0.01), "right")
        right["warn"] = fuzz.trapmf(right.universe, warn_range)
        right["close"] = fuzz.trimf(right.universe, close_range)
        right["medium"] = fuzz.trimf(right.universe, medium_range)
        right["far"] = fuzz.trapmf(right.universe, far_range)

        spin = ctrl.Consequent(np.arange(-3.1415, 3.1415, 0.0174), "spin")
        spin["hard_left"] = fuzz.trimf(spin.universe, [-2.356, -1.5707, -0.7853]) # 90 deg
        spin["left"] = fuzz.trimf(spin.universe, [-1.5707, -0.7853, 0]) # 45 deg
        spin["no_spin"] = fuzz.trimf(spin.universe, [-0.0174, 0, 0.0174]) # 0 deg
        spin["right"] = fuzz.trimf(spin.universe, [0 , 0.7853, 1.5707]) # 45 deg
        spin["hard_right"] = fuzz.trimf(spin.universe, [0.7853, 1.5707, 2.356]) # 90 deg

        drive = ctrl.Consequent(np.arange(-0.25, 2.0, 0.01), "drive")
        drive["reverse"] = fuzz.trimf(drive.universe, [-0.25, -0.25, 0])
        drive["short_forward"] = fuzz.trimf(drive.universe, [0, 0.25, 0.5])
        drive["medium_forward"] = fuzz.trimf(drive.universe, [0.25, 0.75, 1.25])
        drive["long_forward"] = fuzz.trimf(drive.universe, [0.75, 1.75, 1.75])

        rules = []

        # drive rules
        rules.append(ctrl.Rule(left["warn"] | slight_left["warn"] | front["warn"] | slight_right["warn"] | 
                               right["warn"] | front["close"] | slight_left["close"] | slight_right["close"] , 
                               drive["reverse"]))
        
        rules.append(ctrl.Rule(left["close"] | right["close"], drive["short_forward"]))

        rules.append(ctrl.Rule(front["medium"] | slight_left["medium"] | slight_right["medium"], drive["medium_forward"]))

        rules.append(ctrl.Rule(front["far"] | slight_left["far"] | slight_right["far"], drive["long_forward"]))

        # spin rules
        rules.append(ctrl.Rule((front["far"] | front["medium"]) & 
                               (slight_left["far"] | slight_left["medium"]) & 
                               (slight_right["far"] | slight_right["medium"]), 
                               spin["no_spin"]))

        rules.append(ctrl.Rule((left["far"] | left["medium"]) & (front["close"] | slight_right["close"]), 
                               spin["hard_left"]))

        rules.append(ctrl.Rule((right["far"] | right["medium"]) & (front["close"] | slight_left["close"]), 
                               spin["hard_right"]))
        
        rules.append(ctrl.Rule((left["far"] | left["medium"]) & (right["close"] | right["warn"]), 
                               spin["left"]))

        rules.append(ctrl.Rule((right["far"] | right["medium"]) & (left["close"] | left["warn"]), 
                               spin["right"]))

        rules.append(ctrl.Rule(front["warn"] | slight_left["warn"] | slight_right["warn"], spin["no_spin"]))

        rules.append(ctrl.Rule(right["warn"] & left["warn"], spin["no_spin"]))

        self.fuzzy_controller = ctrl.ControlSystem(rules)


    def fuzzy_compute(self, scan_results):

        sim = ctrl.ControlSystemSimulation(self.fuzzy_controller)

        sim.input["left"] = scan_results[0]
        sim.input["slight_left"] = scan_results[1]
        sim.input["front"] = scan_results[2]
        sim.input["slight_right"] = scan_results[3]
        sim.input["right"] = scan_results[4]

        sim.compute()

        return sim.output

    def send_spin_goal(self, angle):

        msg = Spin.Goal()
        msg.angle = angle
        self.spin_action_client.wait_for_server()
        future = self.spin_action_client.send_goal_async(msg)
        future.add_done_callback(self.spin_response_callback)

    def spin_response_callback(self, future):

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        future = goal_handle.get_result_async()
        future.add_done_callback(self.spin_result_callback)

    def spin_result_callback(self, future):

        self.callback()


    def send_drive_goal(self, distance):

        msg = Drive.Goal()
        msg.distance = distance
        self.drive_action_client.wait_for_server()
        future = self.drive_action_client.send_goal_async(msg)
        future.add_done_callback(self.drive_response_callback)

    def drive_response_callback(self, future):

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        future = goal_handle.get_result_async()
        future.add_done_callback(self.drive_result_callback)

    def drive_result_callback(self, future):

        self.callback()

def main():
    rclpy.init()
    node = WanderingNode()
    rclpy.spin(node)
    rclpy.shutdown()

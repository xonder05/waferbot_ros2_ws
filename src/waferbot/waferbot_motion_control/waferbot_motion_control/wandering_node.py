import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Range
from action_msgs.msg import GoalStatus
from waferbot_msgs.action import Spin, Drive

import simpful as sf

class WanderingNode(Node):
    def __init__(self):
        super().__init__("wandering_node")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("update_rate", rclpy.Parameter.Type.INTEGER),
                ("servo_move_time", rclpy.Parameter.Type.DOUBLE),
            ]
        )
        self.update_rate = self.get_parameter("update_rate").get_parameter_value().integer_value
        self.servo_move_time = self.get_parameter("servo_move_time").get_parameter_value().double_value

        self.range_subscriber = self.create_subscription(Range, "range", self.range_callback, 10)
        self.servo_publisher = self.create_publisher(Float64MultiArray, "forward_command_controller/commands", 10)
        self.spin_action_client = ActionClient(self, Spin, "spin")
        self.drive_action_client = ActionClient(self, Drive, "drive")

        self.next_state = "Start"
        self.scan_angles = [-0.7853, -0.3927, 0, 0.3927, 0.7853]
        self.scan_results = []
        self.scan_index = 0
        self.scan_save_distance = False
        self.drive_goal_handle = None
        self.fuzzy_init()

        # start fsm execution
        self.sleep_duration = 1 / self.update_rate
        self.timer = self.create_timer(self.sleep_duration, self.fsm)

        self.get_logger().info("InitDone")

    def range_callback(self, msg):
        """Callback function for the ultrasonic range messages"""

        if msg.range < 0.25 and self.drive_goal_handle is not None and self.drive_goal_handle.status == (GoalStatus.STATUS_ACCEPTED or GoalStatus.STATUS_EXECUTING):
            self.drive_goal_handle.cancel_goal_async()

        if self.scan_save_distance == True:
            self.scan_save_distance = False
            self.scan_results.append(msg.range)

#--------------------------------------- FSM ---------------------------------------

    def fsm(self):
        """Main function of this node, implements finite state machine"""
        
        # initialize / reset
        if self.next_state == "Start":
            self.scan_results = []
            self.scan_index = 0
            self.next_state = "Scan_Move"
            return

        # move servo
        if self.next_state == "Scan_Move":

            angle = self.scan_angles[self.scan_index]

            msg = Float64MultiArray()
            msg.data = [angle]
            self.servo_publisher.publish(msg)

            self.scan_index += 1

            self.timer.timer_period_ns = 1_000_000_000
            self.timer.reset()

            self.next_state = "Scan_Scan"
            
            return

        # get distance reading
        if self.next_state == "Scan_Scan":

            if self.timer.timer_period_ns == 1_000_000_000:

                self.timer.timer_period_ns = 50_000_000
                self.timer.reset()

            if len(self.scan_results) <= self.scan_index:
                self.scan_save_distance = True

            else:
                self.next_state = "Scan_Done"

            return

        # repeat or continue
        if self.next_state == "Scan_Done":
            
            if self.scan_index < len(self.scan_angles):
                self.next_state = "Scan_Move"
            
            else:
                msg = Float64MultiArray()
                msg.data = [0]
                self.servo_publisher.publish(msg)

                self.next_state = "Fuzzy"

            return

        # call fuzzy and move
        if self.next_state == "Fuzzy":

            fuzzy_result = self.fuzzy_compute(self.scan_results)

            if fuzzy_result["action"] < 0:
                self.get_logger().info(f"spinning {fuzzy_result["spin"]}")
                self.send_spin_goal(fuzzy_result["spin"])

            else:
                self.get_logger().info(f"driving {fuzzy_result["drive"]}")
                self.send_drive_goal(fuzzy_result["drive"])

            self.next_state = "Start"
            self.timer.cancel()
            return

#--------------------------------------- Fuzzy ---------------------------------------

    def fuzzy_init(self):
        """Initializes variables and rules for fuzzy inference"""
        self.FS = sf.FuzzySystem()

        # inputs
        warn_range = [0, 0, 0.24, 0.25]
        close_range = [0, 0.5, 1.0]
        medium_range = [0.5, 1.0, 1.5]
        far_range = [1.0, 2.0, 10, 10]

        self.FS.add_linguistic_variable("left", sf.LinguisticVariable([
            sf.TrapezoidFuzzySet(warn_range[0], warn_range[1], warn_range[2], warn_range[3], term="warn"),
            sf.TriangleFuzzySet(close_range[0], close_range[1], close_range[2], term="close"),
            sf.TriangleFuzzySet(medium_range[0], medium_range[1], medium_range[2], term="medium"),
            sf.TrapezoidFuzzySet(far_range[0], far_range[1], far_range[2], far_range[3], term="far"),
        ]))

        self.FS.add_linguistic_variable("slight_left", sf.LinguisticVariable([
            sf.TrapezoidFuzzySet(warn_range[0], warn_range[1], warn_range[2], warn_range[3], term="warn"),
            sf.TriangleFuzzySet(close_range[0], close_range[1], close_range[2], term="close"),
            sf.TriangleFuzzySet(medium_range[0], medium_range[1], medium_range[2], term="medium"),
            sf.TrapezoidFuzzySet(far_range[0], far_range[1], far_range[2], far_range[3], term="far"),
        ]))

        self.FS.add_linguistic_variable("front", sf.LinguisticVariable([
            sf.TrapezoidFuzzySet(warn_range[0], warn_range[1], warn_range[2], warn_range[3], term="warn"),
            sf.TriangleFuzzySet(close_range[0], close_range[1], close_range[2], term="close"),
            sf.TriangleFuzzySet(medium_range[0], medium_range[1], medium_range[2], term="medium"),
            sf.TrapezoidFuzzySet(far_range[0], far_range[1], far_range[2], far_range[3], term="far"),
        ]))

        self.FS.add_linguistic_variable("slight_right", sf.LinguisticVariable([
            sf.TrapezoidFuzzySet(warn_range[0], warn_range[1], warn_range[2], warn_range[3], term="warn"),
            sf.TriangleFuzzySet(close_range[0], close_range[1], close_range[2], term="close"),
            sf.TriangleFuzzySet(medium_range[0], medium_range[1], medium_range[2], term="medium"),
            sf.TrapezoidFuzzySet(far_range[0], far_range[1], far_range[2], far_range[3], term="far"),
        ]))

        self.FS.add_linguistic_variable("right", sf.LinguisticVariable([
            sf.TrapezoidFuzzySet(warn_range[0], warn_range[1], warn_range[2], warn_range[3], term="warn"),
            sf.TriangleFuzzySet(close_range[0], close_range[1], close_range[2], term="close"),
            sf.TriangleFuzzySet(medium_range[0], medium_range[1], medium_range[2], term="medium"),
            sf.TrapezoidFuzzySet(far_range[0], far_range[1], far_range[2], far_range[3], term="far"),
        ]))

        # outputs
        self.FS.set_crisp_output_value("spin_left", -1.5707)
        self.FS.set_crisp_output_value("spin_slight_left", -0.7853)
        self.FS.set_crisp_output_value("spin_slight_right", 0.7853)
        self.FS.set_crisp_output_value("spin_right", 1.5707)

        self.FS.set_crisp_output_value("drive_stop", 0)
        self.FS.set_crisp_output_value("drive_short", 0.25)
        self.FS.set_crisp_output_value("drive_medium", 0.75)
        self.FS.set_crisp_output_value("drive_far", 1.75)

        self.FS.set_crisp_output_value("spin", -1)
        self.FS.set_crisp_output_value("drive", 1)

        # rules
        rules = []

        # action rules
        rules.append("IF (left IS warn) THEN (action IS spin)")
        rules.append("IF (slight_left IS warn) THEN (action IS spin)")
        rules.append("IF (front IS warn) THEN (action IS spin)")
        rules.append("IF (slight_right IS warn) THEN (action IS spin)")
        rules.append("IF (right IS warn) THEN (action IS spin)")

        # rules.append("IF (left IS close) THEN (action IS spin)")
        rules.append("IF (slight_left IS close) THEN (action IS spin)")
        rules.append("IF (front IS close) THEN (action IS spin)")
        rules.append("IF (slight_right IS close) THEN (action IS spin)")
        # rules.append("IF (right IS close) THEN (action IS spin)")

        rules.append("""
                     IF ((slight_left IS medium) OR (front IS medium) OR (slight_right IS medium) OR
                     (slight_left IS far) OR (front IS far) OR (slight_right IS far))
                     THEN (action IS drive)
        """)

        # drive rules
        rules.append("IF (left IS warn) THEN (drive IS drive_stop)")
        rules.append("IF (slight_left IS warn) THEN (drive IS drive_stop)")
        rules.append("IF (front IS warn) THEN (drive IS drive_stop)")
        rules.append("IF (slight_right IS warn) THEN (drive IS drive_stop)")
        rules.append("IF (right IS warn) THEN (drive IS drive_stop)")

        rules.append("IF (left IS close) THEN (drive IS drive_short)")
        rules.append("IF (slight_left IS close) THEN (drive IS drive_short)")
        rules.append("IF (front IS close) THEN (drive IS drive_short)")
        rules.append("IF (slight_right IS close) THEN (drive IS drive_short)")
        rules.append("IF (right IS close) THEN (drive IS drive_short)")

        rules.append("IF (left IS medium) THEN (drive IS drive_medium)")
        rules.append("IF (slight_left IS medium) THEN (drive IS drive_medium)")
        rules.append("IF (front IS medium) THEN (drive IS drive_medium)")
        rules.append("IF (slight_right IS medium) THEN (drive IS drive_medium)")
        rules.append("IF (right IS medium) THEN (drive IS drive_medium)")

        rules.append("IF (left IS far) THEN (drive IS drive_far)")
        rules.append("IF (slight_left IS far) THEN (drive IS drive_far)")
        rules.append("IF (front IS far) THEN (drive IS drive_far)")
        rules.append("IF (slight_right IS far) THEN (drive IS drive_far)")
        rules.append("IF (right IS far) THEN (drive IS drive_far)")

        # spin rules
        rules.append("""
                     IF (((left IS warn) OR (left IS close)) AND ((slight_left IS warn) OR (slight_left IS close)) AND
                     ((front IS warn) OR (front IS close)) AND ((slight_right IS warn) OR (slight_right IS close)) AND 
                     ((right IS warn) OR (right IS close))) THEN (spin IS spin_right)
        """)

        rules.append("""
                     IF (((left IS far) OR (left IS medium)) AND ((front IS close) OR (slight_right IS close)))
                     THEN (spin IS spin_left)
        """)

        rules.append("""
                     IF (((right IS far) OR (right IS medium)) AND ((front IS close) OR (slight_left IS close)))
                     THEN (spin IS spin_right)
        """)

        rules.append("""
                     IF (((left IS far) OR (left IS medium)) AND ((right IS close) OR (right IS warn)))
                     THEN (spin IS spin_slight_left)
        """)

        rules.append("""
                     IF (((right IS far) OR (right IS medium)) AND ((left IS close) OR (left IS warn)))
                     THEN (spin IS spin_slight_right)
        """)

        self.FS.add_rules(rules)


    def fuzzy_compute(self, scan_results):
        """Initializes variables and rules for fuzzy inference"""
        fuzzy_result = []
        
        if self.FS is not None:
            self.FS.set_variable("left", scan_results[0])
            self.FS.set_variable("slight_left", scan_results[1])
            self.FS.set_variable("front", scan_results[2])
            self.FS.set_variable("slight_right", scan_results[3])
            self.FS.set_variable("right", scan_results[4])

            fuzzy_result = self.FS.Sugeno_inference()
        
        return fuzzy_result

#--------------------------------------- Actions ---------------------------------------

    def send_spin_goal(self, angle):
        """Calls spin action of the motion_executor node"""
        self.spin_action_client.wait_for_server()

        msg = Spin.Goal()
        msg.angle = angle
        future = self.spin_action_client.send_goal_async(msg)

        future.add_done_callback(self.spin_response_callback)


    def spin_response_callback(self, future):
        """If goal got accepted ask for result"""
        goal_handle = future.result()

        # rejected
        if not goal_handle.accepted:
            self.get_logger().info("Spin goal rejected")
            return
        
        # accepted
        future = goal_handle.get_result_async()
        future.add_done_callback(self.spin_result_callback)


    def spin_result_callback(self, future):
        """The result does not matter, restarts timer to continue FSM execution"""
        self.timer = self.create_timer(self.sleep_duration, self.fsm)


    def send_drive_goal(self, distance):
        """Calls drive action of the motion_executor node"""
        self.drive_action_client.wait_for_server()

        msg = Drive.Goal()
        msg.distance = distance
        future = self.drive_action_client.send_goal_async(msg)

        future.add_done_callback(self.drive_response_callback)


    def drive_response_callback(self, future):
        """If goal got accepted ask for result"""
        self.drive_goal_handle = future.result()

        # rejected        
        if not self.drive_goal_handle.accepted:
            self.get_logger().info("Drive goal rejected")
            return

        # accepted
        future = self.drive_goal_handle.get_result_async()
        future.add_done_callback(self.drive_result_callback)


    def drive_result_callback(self, future):
        """The result does not matter, restarts timer to continue FSM execution"""
        self.timer = self.create_timer(self.sleep_duration, self.fsm)


def main():
    rclpy.init()
    node = WanderingNode()
    rclpy.spin(node)
    rclpy.shutdown()

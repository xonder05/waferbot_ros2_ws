#!/usr/bin/env python3

import uuid, json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from rmf_task_msgs.msg import ApiRequest, ApiResponse

class TaskRequester(Node):
    def __init__(self):
        super().__init__("task_requester")

        self.fleet = self.declare_parameter("fleet", "").value
        self.robot = self.declare_parameter("robot", "").value
        self.start_delay_ms = self.declare_parameter("start_delay_ms", 0).value
        self.task = self.declare_parameter("task", "go_to_place").value
        self.place = self.declare_parameter("place", "").value
        self.places = self.declare_parameter("places", ["", ""]).value
        self.rounds = self.declare_parameter("rounds", 1).value
        self.task_id = self.declare_parameter("task_id", "").value

        qos = QoSProfile(
            depth = 1,
            durability = DurabilityPolicy.TRANSIENT_LOCAL,
            reliability = ReliabilityPolicy.RELIABLE,
            history = HistoryPolicy.KEEP_LAST
        )

        self.request_publisher = self.create_publisher(ApiRequest, "/task_api_requests", qos)
        self.response_subscriber = self.create_subscription(ApiResponse, "/task_api_responses", self.callback, 10)

        if self.task == "go_to_place":
            self.dispatch_go_to_place()
        elif self.task == "patrol":
            self.dispatch_patrol()
        elif self.task == "delivery":
            self.dispatch_delivery()
        elif self.task == "clean":
            self.dispatch_clean()
        elif self.task == "cancel":
            self.cancel_task()
        else:
            self.get_logger(f"Unknown task {self.task}, exiting")
            rclpy.shutdown()


    def dispatch_go_to_place(self):

        msg = ApiRequest()
        msg.request_id = "go_to_place_" + str(uuid.uuid4())

        json_msg = {
            "type": "dispatch_task_request" if self.fleet == "" and self.robot == "" else "robot_task_request",
            "fleet": self.fleet,
            "robot": self.robot,
            "request": {
                "category": "compose",
                "unix_millis_request_time": self.get_clock().now().nanoseconds // 1_000_000,
                "unix_millis_earliest_start_time": self.get_clock().now().nanoseconds // 1_000_000 + self.start_delay_ms,
                "description": {
                    "category": "go_to_place",
                    "phases": [
                        {
                            "activity": {
                                "category": "go_to_place",
                                "description": {
                                    "waypoint": self.place
                                }
                            }
                        }
                    ]
                }
            }
        }
        msg.json_msg = json.dumps(json_msg)

        self.request_publisher.publish(msg)
        self.get_logger().info(f"Sending request: \n{json.dumps(json_msg, indent=4)}")
        self.request_id = msg.request_id


    def dispatch_patrol(self):

        msg = ApiRequest()
        msg.request_id = "patrol_" + str(uuid.uuid4())

        json_msg = {
            "type": "dispatch_task_request" if self.fleet == "" or self.robot == "" else "robot_task_request",
            "fleet": self.fleet,
            "robot": self.robot,
            "request": {
                "category": "patrol",
                "unix_millis_request_time": self.get_clock().now().nanoseconds // 1_000_000,
                "unix_millis_earliest_start_time": self.get_clock().now().nanoseconds // 1_000_000 + self.start_delay_ms,
                "description": {
                    "places": self.places,
                    "rounds": self.rounds
                }             
            }
        }
        msg.json_msg = json.dumps(json_msg)

        self.request_publisher.publish(msg)
        self.get_logger().info(f"Sending request: \n{json.dumps(json_msg, indent=4)}")
        self.request_id = msg.request_id


    def dispatch_delivery(self):

        msg = ApiRequest()
        msg.request_id = "delivery_" + str(uuid.uuid4())
        
        json_msg = {
            "type": "dispatch_task_request" if self.fleet == "" and self.robot == "" else "robot_task_request",
            "fleet": self.fleet,
            "robot": self.robot,
            "request": {
                "category": "delivery",
                "unix_millis_request_time": self.get_clock().now().nanoseconds // 1_000_000,
                "unix_millis_earliest_start_time": self.get_clock().now().nanoseconds // 1_000_000 + self.start_delay_ms,
                "description": {
                    "pickup": {
                        "place": self.places[0],
                        "handler": "mock_dispenser",
                        "payload": []
                    },
                    "dropoff": {
                        "place": self.places[1],
                        "handler": "mock_ingestor",
                        "payload": []
                    }
                }
            }
        }
        msg.json_msg = json.dumps(json_msg)

        self.request_publisher.publish(msg)
        self.get_logger().info(f"Sending request: \n{json.dumps(json_msg, indent=4)}")
        self.request_id = msg.request_id


    def dispatch_clean(self):

        msg = ApiRequest()
        msg.request_id = "clean_" + str(uuid.uuid4())

        json_msg = {
            "type": "dispatch_task_request" if self.fleet == "" and self.robot == "" else "robot_task_request",
            "fleet": self.fleet,
            "robot": self.robot,
            "request": {
                "category": "compose",
                "unix_millis_request_time": self.get_clock().now().nanoseconds // 1_000_000,
                "unix_millis_earliest_start_time": self.get_clock().now().nanoseconds // 1_000_000 + self.start_delay_ms,
                "description": {
                    "category": "clean",
                    "phases": [
                        {
                            "activity": {
                                "category": "go_to_place",
                                "description": {
                                    "place": self.place
                                }
                            },
                        },
                        {
                           "activity": {
                                "category": "perform_action",
                                "description": {
                                    "unix_millis_action_duration_estimate": 60000,
                                    "category": "clean",
                                    "expected_finish_location": self.place,
                                    "description": {
                                        "zone": self.place
                                    },
                                    "use_tool_sink": True,                                
                                }
                            }   
                        }
                    ],
                }
            }
        }
        msg.json_msg = json.dumps(json_msg)

        self.request_publisher.publish(msg)
        self.get_logger().info(f"Sending request: \n{json.dumps(json_msg, indent=4)}")
        self.request_id = msg.request_id


    def cancel_task(self):

        msg = ApiRequest()
        msg.request_id = "cancel_" + str(uuid.uuid4())

        json_msg = {
            "type": "cancel_task_request",
            "task_id": self.task_id,
        }
        msg.json_msg = json.dumps(json_msg)

        self.request_publisher.publish(msg)
        self.get_logger().info(f"Sending request: \n{json.dumps(json_msg, indent=2)}")
        self.request_id = msg.request_id


    def callback(self, msg: ApiResponse):
        if msg.request_id == self.request_id:
            self.get_logger().info(f"Got response: \n{json.dumps(json.loads(msg.json_msg), indent=2)}")
            rclpy.shutdown()


def main():
    rclpy.init()
    node = TaskRequester()
    rclpy.spin(node)

if __name__ == "__main__":
    main()

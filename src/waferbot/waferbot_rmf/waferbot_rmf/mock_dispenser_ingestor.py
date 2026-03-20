#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from rmf_dispenser_msgs.msg import DispenserRequest, DispenserResult
from rmf_ingestor_msgs.msg import IngestorRequest, IngestorResult

class MockDispenserIngestor(Node):
    def __init__(self):
        super().__init__("mock_dispenser_ingestor")
        
        self.dispenser_subscriber = self.create_subscription(DispenserRequest, "/dispenser_requests", self.dispenser_callback, 10)
        self.dispenser_publisher = self.create_publisher(DispenserResult, "/dispenser_results", 10)
        self.ingestor_subscriber = self.create_subscription(IngestorRequest, "/ingestor_requests", self.ingestor_callback, 10)
        self.ingestor_publisher = self.create_publisher(IngestorResult, "/ingestor_results", 10)

        self.get_logger().info("InitDone")


    def dispenser_callback(self, msg_in: DispenserRequest):

        self.get_logger().info("Dispenser")

        msg_out = DispenserResult()
        msg_out.request_guid = msg_in.request_guid
        msg_out.source_guid = msg_in.target_guid
        msg_out.status = DispenserResult.SUCCESS

        self.dispenser_publisher.publish(msg_out)


    def ingestor_callback(self, msg_in: IngestorRequest):

        self.get_logger().info("Ingestor")

        msg_out = IngestorResult()
        msg_out.request_guid = msg_in.request_guid
        msg_out.source_guid = msg_in.target_guid
        msg_out.status = DispenserResult.SUCCESS

        self.ingestor_publisher.publish(msg_out)


def main():
    rclpy.init()
    node = MockDispenserIngestor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

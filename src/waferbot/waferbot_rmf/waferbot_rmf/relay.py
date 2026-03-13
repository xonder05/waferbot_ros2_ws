#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class RelayNode(Node):
    def __init__(self):
        super().__init__("relay_node")
        
        qos = QoSProfile(
            depth = 10,
            durability = DurabilityPolicy.TRANSIENT_LOCAL,
            reliability = ReliabilityPolicy.RELIABLE
        )

        self.map_markers_subscriber = self.create_subscription(MarkerArray, "/map_markers", self.map_callback, qos)
        self.fleet_markers_subscriber = self.create_subscription(MarkerArray, "/fleet_markers", self.fleet_callback, 10)
        self.schedule_markers_subscriber = self.create_subscription(MarkerArray, "/schedule_markers", self.schedule_callback, 10)
        self.map_markers_publisher = self.create_publisher(MarkerArray, "/map_markers_new", qos)
        self.fleet_markers_publisher = self.create_publisher(MarkerArray, "/fleet_markers_new", 10)
        self.schedule_markers_publisher = self.create_publisher(MarkerArray, "/schedule_markers_new", 10)

        self.broadcaster = StaticTransformBroadcaster(self)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "rmf_map"
        t.child_frame_id = "map"

        t.transform.translation.x = 7.35
        t.transform.translation.y = -11.75

        self.broadcaster.sendTransform(t)

        self.get_logger().info("InitDone")

    def map_callback(self, msg_in):
        
        for marker in msg_in.markers:
            marker.header.frame_id = "rmf_map"

        self.map_markers_publisher.publish(msg_in)

    def fleet_callback(self, msg_in):
        
        for marker in msg_in.markers:
            marker.header.frame_id = "rmf_map"

        self.fleet_markers_publisher.publish(msg_in)

    def schedule_callback(self, msg_in):
        
        for marker in msg_in.markers:
            marker.header.frame_id = "rmf_map"

        self.schedule_markers_publisher.publish(msg_in)

def main():
    rclpy.init()
    node = RelayNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
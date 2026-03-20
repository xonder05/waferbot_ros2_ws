#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import MarkerArray

class RelayNode(Node):
    def __init__(self):
        super().__init__("rmf_visualization_fixer")
        
        self.origin_offset_x = self.declare_parameter("origin_offset_x", 0.0).value
        self.origin_offset_y = self.declare_parameter("origin_offset_y", 0.0).value
        self.schedule_marker_inner_size = self.declare_parameter("schedule_marker_inner_size", 1.0).value
        self.schedule_marker_outer_size = self.declare_parameter("schedule_marker_outer_size", 1.0).value

        qos = QoSProfile(
            depth = 10,
            durability = DurabilityPolicy.TRANSIENT_LOCAL,
            reliability = ReliabilityPolicy.RELIABLE
        )

        self.broadcaster = StaticTransformBroadcaster(self)
        self.map_markers_subscriber = self.create_subscription(MarkerArray, "/map_markers", self.map_callback, qos)
        self.schedule_markers_subscriber = self.create_subscription(MarkerArray, "/schedule_markers", self.schedule_callback, 10)
        self.map_markers_publisher = self.create_publisher(MarkerArray, "/map_markers_new", qos)
        self.schedule_markers_publisher = self.create_publisher(MarkerArray, "/schedule_markers_new", 10)

        # publish static transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "rmf_map"
        t.child_frame_id = "map"
        t.transform.translation.x = self.origin_offset_x
        t.transform.translation.y = self.origin_offset_y
        self.broadcaster.sendTransform(t)

        self.get_logger().info("InitDone")


    def map_callback(self, msg):
        
        for marker in msg.markers:
            marker.header.frame_id = "rmf_map"

        self.map_markers_publisher.publish(msg)


    def schedule_callback(self, msg):

        for marker in msg.markers:
            marker.header.frame_id = "rmf_map"

            if "participant location" in marker.ns:
                if marker.id == 0:
                    marker.scale.x = self.schedule_marker_inner_size
                    marker.scale.y = self.schedule_marker_inner_size
                    marker.scale.z = self.schedule_marker_inner_size
                else:
                    marker.scale.x = self.schedule_marker_outer_size
                    marker.scale.y = self.schedule_marker_outer_size
                    marker.scale.z = self.schedule_marker_outer_size

        self.schedule_markers_publisher.publish(msg)


def main():
    rclpy.init()
    node = RelayNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

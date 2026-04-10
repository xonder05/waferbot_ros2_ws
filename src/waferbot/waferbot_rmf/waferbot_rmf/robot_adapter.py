#!/usr/bin/env python3

# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rmf_adapter.easy_full_control as rmf_easy

from rclpy.action import ActionClient
from rclpy.time import Duration

import tf2_ros

from nav2_msgs.action import NavigateToPose, DockRobot
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus

import transforms3d


class RobotUpdateData:
    ''' Update data for a single robot. '''
    def __init__(self,
                 robot_name: str,
                 map: str,
                 position: list[float],
                 battery_soc: float,
                 requires_replan: bool | None = None):
        self.robot_name = robot_name
        self.position = position
        self.map = map
        self.battery_soc = battery_soc
        self.requires_replan = requires_replan


class TransformListener:
    def __init__(self, node, robot_name):
        self.node = node
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
        self.robot_name = robot_name


    def get_position(self):

        t = self.tf_buffer.lookup_transform(
            "map", self.robot_name + "/base_link", self.node.get_clock().now()- Duration(seconds=1.0)
        )

        x = t.transform.translation.x
        y = t.transform.translation.y
        
        q = t.transform.rotation
        quat = [q.w, q.x, q.y, q.z]
        _, _, yaw = transforms3d.euler.quat2euler(quat)

        return (x, y, yaw)


class Nav2RobotAdapter:
    def __init__(self, name: str, configuration, node, fleet_handle):
        
        self.name = name
        self.execution = None
        self.destination = None
        self.queued_execution = None
        self.queued_destination = None
        self.update_handle = None
        self.configuration = configuration
        self.node = node
        self.fleet_handle = fleet_handle
        self.tf = TransformListener(self.node, self.name)
        self.navigate_to_pose_client = ActionClient(self.node, NavigateToPose, self.name + "/navigate_to_pose")
        self.dock_robot_client = ActionClient(self.node, DockRobot, self.name + "/dock_robot")
        self.navigate_timeout = None


    def get_data(self):
        
        map = "L1"
        position = self.tf.get_position()
        battery_soc = 1.0
        
        if not (map is None or position is None or battery_soc is None):
            return RobotUpdateData(self.name, map, position, battery_soc)
        return None


    def update(self, state):
        activity_identifier = None
        if self.execution:
            activity_identifier = self.execution.identifier

        self.update_handle.update(state, activity_identifier)


    def make_callbacks(self):
        callbacks = rmf_easy.RobotCallbacks(
            lambda destination, execution: self.navigate(
                destination, execution
            ),
            lambda activity: self.stop(activity),
            lambda category, description, execution: self.execute_action(
                category, description, execution
            )
        )

        return callbacks


    def navigate(self, destination, execution):

        self.queued_execution = execution
        self.queued_destination = destination

        if (self.execution is None and self.destination is None):
            if (self.queued_destination.dock is None):
                self.navigate_to_pose_request()
            else:
                self.dock_robot_request()


    def navigate_to_pose_request(self):

        self.execution = self.queued_execution
        self.queued_execution = None
        self.destination = self.queued_destination
        self.queued_destination = None

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(self.destination.position[0])
        goal_msg.pose.pose.position.y = float(self.destination.position[1])
        goal_msg.pose.pose.position.z = 0.0

        self.navigate_to_pose_client.wait_for_server()
        self.response_future = self.navigate_to_pose_client.send_goal_async(goal_msg)
        self.response_future.add_done_callback(self.navigate_to_pose_response_callback)

        self.node.get_logger().info(f'Commanding [{self.name}] to navigate to {self.destination.position}')


    def navigate_to_pose_response_callback(self, response_future):

        self.goal_handle = response_future.result()

        if self.goal_handle.accepted:
            self.result_future = self.goal_handle.get_result_async()
            self.result_future.add_done_callback(self.navigate_to_pose_result_callback)
        
        else:
            self.node.get_logger().info(f"[{self.name}] refused navigation request")
            self.execution.finished()
            self.execution = None


    def navigate_to_pose_result_callback(self, result_future):

        result = result_future.result()

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info(f"[{self.name}] successfully navigated to requested position")
            self.execution.finished()
            self.execution = None
            self.destination = None

        elif result.status == GoalStatus.STATUS_CANCELED:
            self.node.get_logger().info(f"[{self.name}] navigation successfully canceled")
            self.execution.finished()
            self.execution = None
            self.destination = None

        else:
            self.node.get_logger().info(f"[{self.name}] failed to navigate to requested position")
            self.execution.finished()
            self.execution = None
            self.destination = None

        if (self.queued_execution is not None and self.queued_destination is not None):
            if (self.queued_destination.dock is None):
                self.navigate_to_pose_request()
            else:
                self.dock_robot_request()


    def dock_robot_request(self):

        self.execution = self.queued_execution
        self.queued_execution = None
        self.destination = self.queued_destination
        self.queued_destination = None

        goal_msg = DockRobot.Goal()
        goal_msg.use_dock_id = True
        goal_msg.dock_id = self.destination.dock
        goal_msg.dock_type = "simple"
        goal_msg.navigate_to_staging_pose = True
        goal_msg.max_staging_time = 60.0

        self.dock_robot_client.wait_for_server()
        self.response_future = self.dock_robot_client.send_goal_async(goal_msg)
        self.response_future.add_done_callback(self.dock_robot_response_callback)

        self.node.get_logger().info(f'Commanding [{self.name}] to dock at {self.destination.dock}')


    def dock_robot_response_callback(self, response_future):

        self.goal_handle = response_future.result()

        if self.goal_handle.accepted:
            self.result_future = self.goal_handle.get_result_async()
            self.result_future.add_done_callback(self.dock_robot_result_callback)
        
        else:
            self.node.get_logger().info(f"[{self.name}] refused docking request")
            self.execution.finished()
            self.execution = None


    def dock_robot_result_callback(self, result_future):

        result = result_future.result()

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info(f"[{self.name}] successfully docked")
            self.execution.finished()
            self.execution = None
            self.destination = None

        elif result.status == GoalStatus.STATUS_CANCELED:
            self.node.get_logger().info(f"[{self.name}] navigation successfully canceled")
            self.execution.finished()
            self.execution = None
            self.destination = None

        else:
            self.node.get_logger().info(f"[{self.name}] failed to dock")
            self.execution.finished()
            self.execution = None
            self.destination = None

        if (self.queued_execution is not None and self.queued_destination is not None):
            if (self.queued_destination.dock is None):
                self.navigate_to_pose_request()
            else:
                self.dock_robot_request()


    def stop(self, activity):
        if self.execution is not None:
            if self.execution.identifier.is_same(activity):
                self.cancel_goal_request()


    def cancel_goal_request(self):
        if self.goal_handle is not None:
            self.response_future = self.goal_handle.cancel_goal_async()
            self.response_future.add_done_callback(self.cancel_goal_response_callback)


    def cancel_goal_response_callback(self, response_future):
        result = response_future.result()


    def execute_action(self, category: str, description: dict, execution):
        ''' Trigger a custom action you would like your robot to perform.
        You may wish to use RobotAPI.start_activity to trigger different
        types of actions to your robot.'''
        self.execution = execution

        self.node.get_logger().info(f"got action {category}, {description}, {execution}")

        return

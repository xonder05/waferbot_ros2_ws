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

import tf2_ros

import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
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
            "map", self.robot_name + "/base_link", rclpy.time.Time()
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
        self.update_handle = None
        self.configuration = configuration
        self.node = node
        self.fleet_handle = fleet_handle
        self.tf = TransformListener(self.node, self.name)
        self.action_cli = ActionClient(self.node, NavigateToPose, self.name + "/navigate_to_pose")
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

        # callbacks.localize = lambda estimate, execution: self.localize(
        #     estimate, execution
        # )

        return callbacks

    # def localize(self, estimate, execution):
    #     self.node.get_logger().info(
    #         f'Commanding [{self.name}] to change map to'
    #         f' [{estimate.map}]'
    #     )
    #     if self.api.localize(self.name, estimate.position, estimate.map):
    #         self.node.get_logger().info(
    #             f'Localized [{self.name}] on {estimate.map} '
    #             f'at position [{estimate.position}]'
    #         )
    #         execution.finished()
    #     else:
    #         self.node.get_logger().warn(
    #             f'Failed to localize [{self.name}] on {estimate.map} '
    #             f'at position [{estimate.position}]. Requesting replanning...'
    #         )
    #         if self.update_handle is not None and self.update_handle.more() is not None:
    #             self.update_handle.more().replan()


    def navigate(self, destination, execution):

        # self.node.get_logger().info(str(type(destination)))
        # self.node.get_logger().info(str(dir(destination)))

        # todo check destination.dock and if so command nav2 to dock there

        if self.execution is not None:
            self.node.get_logger().info("Refusing navigation command because the previous one is still active")
            return

        self.node.get_logger().info(
            f'Commanding [{self.name}] to navigate to {destination.position} '
            f'on map [{destination.map}]'
        )

        self.execution = execution

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = float(destination.position[0])
        goal_msg.pose.pose.position.y = float(destination.position[1])
        goal_msg.pose.pose.position.z = 0.0

        self.action_cli.wait_for_server()

        self.navigate_goal_future = self.action_cli.send_goal_async(
            goal_msg,
            # feedback_callback=self.navigate_feedback_callback
        )
        self.navigate_goal_future.add_done_callback(self.navigate_goal_response_callback)


    def navigate_goal_response_callback(self, future):

        self.goal_handle = future.result()
        if self.goal_handle.accepted:
            self.navigate_result_future = self.goal_handle.get_result_async()
            self.navigate_result_future.add_done_callback(self.navigate_result_callback)
        else:
            self.execution.finished()
            self.execution = None
            self.node.get_logger().info(f"Execution object nullified, goal refused")


    # def navigate_feedback_callback(self, feedback_msg):
    #     if self.navigate_timeout is None:
    #         self.navigate_timeout = self.node.create_timer(1.0, self.stop)
    #     else:
    #         self.navigate_timeout.reset()


    def navigate_result_callback(self, future):

        result = future.result()

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.execution.finished()
        else:
            self.execution.finished()

        self.execution = None
        self.node.get_logger().info(f"Execution object nullified, result")


    def stop(self, activity):
        execution = self.execution
        if execution is not None:
            if execution.identifier.is_same(activity):
                if self.goal_handle is not None:
                    self.goal_handle.cancel_goal_async()


    def execute_action(self, category: str, description: dict, execution):
        ''' Trigger a custom action you would like your robot to perform.
        You may wish to use RobotAPI.start_activity to trigger different
        types of actions to your robot.'''
        self.execution = execution
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return
    
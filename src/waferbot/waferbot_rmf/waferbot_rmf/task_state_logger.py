#!/usr/bin/env python3

import json

import rclpy
from rclpy.node import Node

from rmf_task_msgs.msg import ApiResponse, DispatchStates
from rmf_fleet_msgs.msg import FleetState

class TaskStateLogger(Node):
    def __init__(self):
        super().__init__("task_state_logger")

        self.response_subscriber = self.create_subscription(ApiResponse, "/task_api_responses", self.response_callback, 10)
        self.response_subscriber = self.create_subscription(DispatchStates, "/dispatch_states", self.dispatch_callback, 10)
        self.response_subscriber = self.create_subscription(FleetState, "/fleet_states", self.fleet_callback, 10)

        self.active_tasks = {}

        self.get_logger().info("InitDone")


    def print_state(self, id):

        state = self.active_tasks.get(id).get("state", "")
        type = self.active_tasks.get(id).get("type", "")
        fleet = self.active_tasks.get(id).get("fleet", "")
        robot = self.active_tasks.get(id).get("robot", "")

        self.get_logger().info(f"Task: {type}, state: {state}, id: {id}, fleet: {fleet}, robot: {robot}")


    def response_callback(self, msg: ApiResponse):
        
        json_msg = None
        try:
            json_msg = json.loads(msg.json_msg)
        except Exception:
            return

        if json_msg.get("success") == True:

            id = json_msg.get("state", {}).get("booking", {}).get("id")
            if id is None: return

            self.active_tasks[id] = {}
            self.active_tasks[id]["state"] = "accepted"

            req_cat = json_msg.get("state", {}).get("category", "")
            req_des_cat = json_msg.get("state", {}).get("detail", {}).get("category", "")

            if req_cat == "compose":
                self.active_tasks[id]["type"] = req_des_cat
            else:
                self.active_tasks[id]["type"] = req_cat

            self.print_state(id)


    def dispatch_callback(self, msg: DispatchStates):

        for task in msg.active:
            if (task.task_id in self.active_tasks and task.assignment.is_assigned and
               self.active_tasks.get(task.task_id, {}).get("state") == "accepted"):

                self.active_tasks[task.task_id]["state"] = "assigned"
                self.active_tasks[task.task_id]["fleet"] = task.assignment.fleet_name
                
                self.print_state(task.task_id)


    def fleet_callback(self, msg: FleetState):

        robots = {}        
        for robot in msg.robots:
            robots[robot.task_id] = robot

        for id, task in list(self.active_tasks.items()):

            if id in robots:
                if task.get("state") == "assigned":
                    self.active_tasks[id]["state"] = "performing"
                    self.active_tasks[id]["robot"] = robots[id].name
                    self.print_state(id)
            
            else:
                if task["state"] == "performing":
                    task["state"] = "finished"
                    self.print_state(id)
                    del self.active_tasks[id]


def main():
    rclpy.init()
    node = TaskStateLogger()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

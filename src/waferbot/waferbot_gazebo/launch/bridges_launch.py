#!/usr/bin/env python3
"""
Filename: bridges_launch.py
Description: Adds bridge node to existing container. 
Author: Daniel Onderka (xonder05)
Date: 10/2025
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare

from ros_gz_bridge.actions import RosGzBridge

def generate_launch_description():
    
    robot_name_arg = DeclareLaunchArgument("robot_name", default_value="waferbot")

    config_file_path = PathJoinSubstitution([
        FindPackageShare("waferbot_gazebo"),
        "config",
        "_bridges.yaml"
    ])

    spawn_bridge_in_container = RosGzBridge (
        
        use_composition=True,
        container_name="ros_gz_bridge_container",
        create_own_container=False,
        
        bridge_name="ros_gz_bridge",
        namespace=LaunchConfiguration("robot_name"),
        config_file=config_file_path,
        extra_bridge_params={"expand_gz_topic_names": True}
    )

    return LaunchDescription([
        robot_name_arg,
        spawn_bridge_in_container,
    ])

#!/usr/bin/env python3
"""
Filename: map_server_launch.py
Description: Launch file for Nav2 map server.
Author: Daniel Onderka (xonder05)
Date: 10/2025
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    robot_name_arg = DeclareLaunchArgument("robot_name", default_value="waferbot")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")
    map_file_arg = DeclareLaunchArgument("map_select", default_value="warehouse")

    config_file_path = PathJoinSubstitution([
        FindPackageShare("waferbot_mapping_bringup"),
        "config",
        "_map_server.yaml"
    ])

    map_file_path = PathJoinSubstitution([
        FindPackageShare("waferbot_mapping_bringup"),
        "maps",
        [LaunchConfiguration("map_select"), "_map.yaml"]
    ])

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        namespace=LaunchConfiguration("robot_name"),
        parameters=[config_file_path, {
            "autostart_node": True,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "yaml_filename": map_file_path
        }],
        remappings=[("map", "/map")],
    )

    return LaunchDescription([
        robot_name_arg,
        use_sim_time_arg,
        map_file_arg,

        map_server,
    ])

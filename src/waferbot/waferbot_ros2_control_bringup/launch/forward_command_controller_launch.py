from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
   
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value="waferbot")

    config_file = os.path.join(
        get_package_share_directory('waferbot_ros2_control_bringup'),
        'config',
        'forward_command_controller.yaml'
    )

    forward_command_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=LaunchConfiguration("robot_name"),
        arguments=[
            "forward_command_controller", 
            "--param-file", config_file,
            "--controller-manager-timeout", "60",
        ],
    )

    return LaunchDescription([
        robot_name_arg,

        forward_command_controller_spawner
    ])
    
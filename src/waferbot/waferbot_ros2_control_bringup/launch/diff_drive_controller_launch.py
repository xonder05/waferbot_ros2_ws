from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config_file = os.path.join(
        get_package_share_directory('waferbot_ros2_control_bringup'),
        'config',
        'diff_drive_controller.yaml'
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            "--param-file", config_file,
            "--controller-manager-timeout", "60",
            ],
    )

    return LaunchDescription([
        diff_drive_controller_spawner,
    ])

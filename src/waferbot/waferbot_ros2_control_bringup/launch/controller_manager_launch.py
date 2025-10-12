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

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value="false")

    config_file = os.path.join(
        get_package_share_directory('waferbot_ros2_control_bringup'),
        'config',
        'controller_manager.yaml'
    )

    manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[config_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        # remappings=[
        #     ('/diff_drive_controller/cmd_vel', '/cmd_vel'),
        #     ('/controller_manager/robot_description', '/robot_description')
        # ]
    )

    return LaunchDescription([
        use_sim_time_arg,

        manager_node,
    ])

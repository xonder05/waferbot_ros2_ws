from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value="false")
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value="waferbot")

    config_file = os.path.join(
        get_package_share_directory('waferbot_ros2_control_bringup'),
        'config',
        'controller_manager.yaml'
    )

    manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=LaunchConfiguration("robot_name"),
        parameters=[config_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_name_arg,

        manager_node,
    ])

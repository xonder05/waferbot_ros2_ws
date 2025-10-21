import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node

def generate_launch_description():
    
    world_select_arg = DeclareLaunchArgument('world_select', default_value='wandering')

    world_file_name = [LaunchConfiguration('world_select'), TextSubstitution(text="_world.sdf")]

    gazebo_world_path = PathJoinSubstitution([
        get_package_share_directory("waferbot_gazebo"),
        "worlds",
        world_file_name
    ])

    simulator = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-r',  gazebo_world_path],
        output='screen',
    )

    #ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock
    clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]
    )

    return LaunchDescription([
        world_select_arg,
        
        simulator,
        clock,
    ])
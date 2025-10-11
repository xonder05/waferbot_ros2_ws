import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution

def generate_launch_description():
    
    world_select_arg = DeclareLaunchArgument('world_select', default_value='wandering')
    world_select_val = LaunchConfiguration('world_select')

    world_file_name = [world_select_val, TextSubstitution(text="_world.sdf")]

    gazebo_world_path = PathJoinSubstitution([
        get_package_share_directory("waferbot_gazebo"),
        "worlds",
        world_file_name
    ])

    simulator = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-r',  gazebo_world_path],
        output='screen',
    )

    return LaunchDescription([
        world_select_arg,
        simulator,
    ])
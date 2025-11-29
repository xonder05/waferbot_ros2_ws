from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    robot_name_arg = DeclareLaunchArgument("robot_name", default_value="waferbot")

    config_file_path = PathJoinSubstitution([
        FindPackageShare("waferbot_motion_control"),
        "config",
        "wandering.yaml"
    ])

    wandering_node = Node(
        package="waferbot_motion_control",
        executable="wandering_node",
        namespace=LaunchConfiguration("robot_name"),
        parameters=[config_file_path, {
            "use_sim_time": True
        }]
    )
    
    return LaunchDescription([
        robot_name_arg,
        wandering_node,
    ])

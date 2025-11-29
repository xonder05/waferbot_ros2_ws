from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    robot_name_arg = DeclareLaunchArgument("robot_name", default_value="waferbot")

    config_file_path = PathJoinSubstitution([
        FindPackageShare("waferbot_sensor_nodes"),
        "config",
        "config.yaml"
    ])

    pi_camera_node = Node(
        package="waferbot_sensor_nodes",
        executable="camera_node",
        namespace=LaunchConfiguration("robot_name"),
        parameters=[config_file_path]
    )
    
    return LaunchDescription([
        robot_name_arg,
        pi_camera_node,
    ])

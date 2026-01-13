from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    robot_name_arg = DeclareLaunchArgument("robot_name", default_value="waferbot")

    config_file_path = PathJoinSubstitution([
        FindPackageShare("waferbot_gazebo"),
        "config",
        "_bridges.yaml"
    ])

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace=LaunchConfiguration("robot_name"),
        parameters=[{
            "expand_gz_topic_names": True,
            "config_file": config_file_path
        }]
    )

    return LaunchDescription([
        robot_name_arg,
        ros_gz_bridge,
    ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    robot_name_arg = DeclareLaunchArgument("robot_name", default_value="waferbot")

    config_file_path = PathJoinSubstitution([
        FindPackageShare("waferbot_control_bringup"),
        "config",
        "_diff_drive_controller.yaml"
    ])

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=LaunchConfiguration("robot_name"),
        arguments=[
            "diff_drive_controller",
            "--param-file", config_file_path,
            "--controller-manager-timeout", "60",
        ],
    )

    return LaunchDescription([
        robot_name_arg,
        diff_drive_controller_spawner,
    ])

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    controller_mangager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("waferbot_ros2_control_bringup"), '/launch', '/controller_manager_launch.py'
        ]),
    )

    diff_drive_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("waferbot_ros2_control_bringup"), '/launch', '/diff_drive_controller_launch.py'
        ]),
    )

    forward_command_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("waferbot_ros2_control_bringup"), '/launch', '/forward_command_controller_launch.py'
        ]),
    )

    joint_state_broadcaster = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("waferbot_ros2_control_bringup"), '/launch', '/joint_state_broadcaster_launch.py'
        ]),
    )

    return LaunchDescription([
        controller_mangager,
        diff_drive_controller,
        forward_command_controller,
        joint_state_broadcaster
    ])
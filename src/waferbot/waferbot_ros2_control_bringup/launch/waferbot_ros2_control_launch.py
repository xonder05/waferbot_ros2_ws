from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    robot_name_arg = DeclareLaunchArgument('robot_name', default_value="waferbot")

    controller_mangager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("waferbot_ros2_control_bringup"), '/launch', '/controller_manager_launch.py'
        ]),
        launch_arguments={"robot_name": LaunchConfiguration("robot_name")}.items()  
    )

    diff_drive_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("waferbot_ros2_control_bringup"), '/launch', '/diff_drive_controller_launch.py'
        ]),
        launch_arguments={"robot_name": LaunchConfiguration("robot_name")}.items()  
    )

    forward_command_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("waferbot_ros2_control_bringup"), '/launch', '/forward_command_controller_launch.py'
        ]),
        launch_arguments={"robot_name": LaunchConfiguration("robot_name")}.items()  
    )

    joint_state_broadcaster = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("waferbot_ros2_control_bringup"), '/launch', '/joint_state_broadcaster_launch.py'
        ]),
        launch_arguments={"robot_name": LaunchConfiguration("robot_name")}.items()  
    )

    return LaunchDescription([
        robot_name_arg,
        
        controller_mangager,
        diff_drive_controller,
        forward_command_controller,
        joint_state_broadcaster
    ])
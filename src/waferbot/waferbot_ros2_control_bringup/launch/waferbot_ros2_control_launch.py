from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    robot_name_arg = DeclareLaunchArgument("robot_name", default_value="waferbot")

    controller_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("waferbot_ros2_control_bringup"), 
                "launch", 
                "controller_manager_launch.py"
            ])
        ),
        launch_arguments=[
            ("robot_name", LaunchConfiguration("robot_name"))            
        ]
    )

    diff_drive_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("waferbot_ros2_control_bringup"), 
                "launch", 
                "diff_drive_controller_launch.py"
            ])
        ),
        launch_arguments=[
            ("robot_name", LaunchConfiguration("robot_name"))            
        ]
    )

    forward_command_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("waferbot_ros2_control_bringup"), 
                "launch", 
                "forward_command_controller_launch.py"
            ])
        ),
        launch_arguments=[
            ("robot_name", LaunchConfiguration("robot_name"))            
        ]
    )

    joint_state_broadcaster = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("waferbot_ros2_control_bringup"), 
                "launch", 
                "joint_state_broadcaster_launch.py"
            ])
        ),
        launch_arguments=[
            ("robot_name", LaunchConfiguration("robot_name"))            
        ]
    )

    return LaunchDescription([
        robot_name_arg,
        controller_manager,
        diff_drive_controller,
        forward_command_controller,
        joint_state_broadcaster
    ])

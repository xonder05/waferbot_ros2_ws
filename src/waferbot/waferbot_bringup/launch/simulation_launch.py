from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    robot_name_arg = DeclareLaunchArgument("robot_name", default_value="waferbot")
    world_select_arg = DeclareLaunchArgument("world_select", default_value="wandering")

    # simulator and world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("waferbot_gazebo"), 
                "launch", 
                "gazebo_launch.py"
            ])
        ),
        launch_arguments=[
            ("world_select", LaunchConfiguration("world_select"))   
        ]
    )

    # robot from description
    robot_spawn = TimerAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("waferbot_gazebo"), 
                        "launch", 
                        "spawn_robot_launch.py"
                    ])
                ),
                launch_arguments=[
                    ("world_select", LaunchConfiguration("world_select"))
                ]
            )
        ],
        period=2.0
    )

    # actuators
    diff_drive_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("waferbot_control_bringup"), 
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
                FindPackageShare("waferbot_control_bringup"), 
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
                FindPackageShare("waferbot_control_bringup"), 
                "launch", 
                "joint_state_broadcaster_launch.py"
            ])
        ),
        launch_arguments=[
            ("robot_name", LaunchConfiguration("robot_name"))            
        ]
    )

    # ros_gz_bridges
    bridges = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("waferbot_gazebo"), 
                "launch", 
                "bridges_launch.py"
            ])
        )
    )

    # helper nodes
    helpers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("waferbot_gazebo"), 
                "launch", 
                "helpers_launch.py"
            ])
        ),
        launch_arguments=[
            ("robot_name", LaunchConfiguration("robot_name"))
        ]
    )

    return LaunchDescription([
        robot_name_arg,
        world_select_arg,
        
        gazebo,
        robot_spawn,
        diff_drive_controller,
        forward_command_controller,
        joint_state_broadcaster,
        bridges,
        helpers,
    ])

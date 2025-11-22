from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    world_select_arg = DeclareLaunchArgument('world_select', default_value='wandering')

    robot_name_arg = DeclareLaunchArgument('robot_name', default_value="waferbot")

    # simulator start on selected world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("waferbot_gazebo"), '/launch', '/gazebo_launch.py'
        ]),
        launch_arguments={'world_select': LaunchConfiguration('world_select')}.items()
    )

    # xacro conversion and robot spawner
    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("waferbot_gazebo"), '/launch', '/robot_spawn_launch.py'
        ]),
        launch_arguments={'world_select': LaunchConfiguration('world_select')}.items()
    )

    # ros2_control
    diff_drive_controller_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("waferbot_ros2_control_bringup"), '/launch', '/diff_drive_controller_launch.py'
        ])
    )

    forward_command_controller_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("waferbot_ros2_control_bringup"), '/launch', '/forward_command_controller_launch.py'
        ])
    )

    # ros_gz_bridges
    bridges = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("waferbot_gazebo"), '/launch', '/bridges_launch.py'
        ])
    )

    # helper nodes
    helpers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("waferbot_gazebo"), '/launch', '/helpers_launch.py'
        ]),
        launch_arguments={'robot_name': LaunchConfiguration('robot_name')}.items()
    )

    return LaunchDescription([
        world_select_arg,
        robot_name_arg,

        gazebo,
        robot_spawn,
        diff_drive_controller_spawn,
        forward_command_controller_spawn,
        bridges,
        helpers,
    ])

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    world_select_arg = DeclareLaunchArgument('world_select', default_value='wandering')
    world_select_val = LaunchConfiguration('world_select')

    # simulator start on selected world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("waferbot_gazebo"), '/launch', '/gazebo_launch.py'
        ]),
        launch_arguments={'world_select': world_select_val}.items()
    )

    # xacro conversion and robot spawner
    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("waferbot_gazebo"), '/launch', '/robot_spawn_launch.py'
        ]),
        launch_arguments={'world_select': world_select_val}.items()
    )

    # spawn controllers (controller manager is started as gazebo plugin)
    ros2_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("waferbot_gazebo"), '/launch', '/ros2_control_launch.py'
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
        ])
    )

    return LaunchDescription([
        world_select_arg,

        gazebo,
        robot_spawn,
        ros2_control,
        bridges,
        helpers,
    ])
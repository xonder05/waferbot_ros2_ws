from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    robot_name_arg = DeclareLaunchArgument("robot_name", default_value="waferbot")

    config_file_path = PathJoinSubstitution([
        FindPackageShare("waferbot_sensors"),
        "config",
        "_ydlidar.yaml"
    ])

    lidar_node = LifecycleNode(
        package="ydlidar_ros2_driver",
        executable="ydlidar_ros2_driver_node",
        name="ydlidar_ros2_driver_node",
        namespace=LaunchConfiguration("robot_name"),
        parameters=[config_file_path],
        emulate_tty=True,
        output='screen',
    )

    return LaunchDescription([
        robot_name_arg,
        lidar_node,
    ])

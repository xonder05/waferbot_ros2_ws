from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    robot_name_arg = DeclareLaunchArgument("robot_name", default_value="waferbot")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")

    config_file_path = PathJoinSubstitution([
        FindPackageShare("waferbot_ros2_control_bringup"),
        "config",
        "controller_manager.yaml"
    ])

    manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=LaunchConfiguration("robot_name"),
        parameters=[config_file_path, {
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }],
    )

    return LaunchDescription([
        robot_name_arg,
        use_sim_time_arg,
        manager_node,
    ])

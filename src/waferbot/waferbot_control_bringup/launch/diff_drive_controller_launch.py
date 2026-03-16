from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from nav2_common.launch import RewrittenYaml

def generate_launch_description():

    robot_name_arg = DeclareLaunchArgument("robot_name", default_value="waferbot")
    enable_odom_tf_arg = DeclareLaunchArgument("enable_odom_tf", default_value="false")

    config_file_path = PathJoinSubstitution([
        FindPackageShare("waferbot_control_bringup"),
        "config",
        "_diff_drive_controller.yaml"
    ])

    param_substitutions = {
        "/**/diff_drive_controller.ros__parameters.enable_odom_tf": LaunchConfiguration("enable_odom_tf"),
    }

    rewritten_config_file = RewrittenYaml(
        source_file=config_file_path,
        param_rewrites=param_substitutions,
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=LaunchConfiguration("robot_name"),
        arguments=[
            "diff_drive_controller",
            "--param-file", rewritten_config_file,
            "--controller-manager-timeout", "60",
        ],
    )

    return LaunchDescription([
        robot_name_arg,
        enable_odom_tf_arg,
        diff_drive_controller_spawner,
    ])

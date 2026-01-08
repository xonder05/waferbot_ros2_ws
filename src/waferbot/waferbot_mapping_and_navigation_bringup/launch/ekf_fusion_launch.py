from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterFile

from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    
    robot_name_arg = DeclareLaunchArgument("robot_name", default_value="waferbot")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")

    config_file_path = PathJoinSubstitution([
        FindPackageShare("waferbot_mapping_and_navigation_bringup"),
        "config",
        "state_estimation.yaml"
    ])

    param_substitutions = {
        "ekf_filter_node.ros__parameters.odom_frame": [LaunchConfiguration("robot_name"), "/odom"],
        "ekf_filter_node.ros__parameters.base_link_frame": [LaunchConfiguration("robot_name"), "/base_link"],
        "ekf_filter_node.ros__parameters.world_frame": [LaunchConfiguration("robot_name"), "/odom"],
    }
    
    namespaced_config_file = ParameterFile(
        RewrittenYaml(
            source_file=config_file_path,
            root_key=LaunchConfiguration("robot_name"),
            param_rewrites=param_substitutions,
        ),
        allow_substs=True
    )

    localization = Node(
        package="robot_localization",
        executable="ekf_node",
        namespace=LaunchConfiguration("robot_name"),
        parameters=[namespaced_config_file, {
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }],
    )

    return LaunchDescription([
        robot_name_arg,
        use_sim_time_arg,
        localization
    ])

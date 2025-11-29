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
        "live_mapping.yaml"
    ])

    param_substitutions = {
        "async_slam_toolbox_node.ros__parameters.odom_frame": [LaunchConfiguration("robot_name"), "/odom"],
        "async_slam_toolbox_node.ros__parameters.base_frame": [LaunchConfiguration("robot_name"), "/base_footprint"],
    }

    namespaced_config_file = ParameterFile(
        RewrittenYaml(
            source_file=config_file_path,
            root_key=LaunchConfiguration("robot_name"),
            param_rewrites=param_substitutions,
        ),
        allow_substs=True
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        namespace=LaunchConfiguration("robot_name"),
        parameters=[{
            "autostart": True, 
            "node_names": ["async_slam_toolbox_node"]
        }],
    )

    async_slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="async_slam_toolbox_node", # required, default name is different than executable name
        namespace=LaunchConfiguration("robot_name"),
        parameters=[namespaced_config_file, {
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "use_lifecycle_manager": True,
        }],
        remappings=[("/scan", "scan"), ("map", "/map")]
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_name_arg,

        lifecycle_manager,
        async_slam_toolbox_node,
    ])

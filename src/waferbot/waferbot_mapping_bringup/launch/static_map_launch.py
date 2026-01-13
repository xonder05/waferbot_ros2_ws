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
    map_file_arg = DeclareLaunchArgument("map_file", default_value="")

    config_file_path = PathJoinSubstitution([
        FindPackageShare("waferbot_mapping_bringup"),
        "config",
        "_static_map.yaml"
    ])

    param_substitutions = {
        "amcl.ros__parameters.base_frame_id": [LaunchConfiguration("robot_name"), "/base_footprint"],
        "amcl.ros__parameters.odom_frame_id": [LaunchConfiguration("robot_name"), "/odom"],
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
            "node_names": ["map_server", "amcl"]
        }],
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        namespace=LaunchConfiguration("robot_name"),
        parameters=[config_file_path, {
            "use_sim_time": LaunchConfiguration("use_sim_time"), 
            "yaml_filename": LaunchConfiguration("map_file")
        }],
        remappings=[("map", "/map")]
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        namespace=LaunchConfiguration("robot_name"),
        parameters=[namespaced_config_file, {
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }],
        remappings=[("/scan", "scan"), ("map", "/map")]
    )

    return LaunchDescription([
        robot_name_arg,
        use_sim_time_arg,
        map_file_arg,

        lifecycle_manager,
        map_server,
        amcl
    ])

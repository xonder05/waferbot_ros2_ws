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
    initial_pose_x_arg = DeclareLaunchArgument("x", default_value="0.0")
    initial_pose_y_arg = DeclareLaunchArgument("y", default_value="0.0")
    initial_pose_z_arg = DeclareLaunchArgument("yaw", default_value="0.0")

    config_file_path = PathJoinSubstitution([
        FindPackageShare("waferbot_mapping_bringup"),
        "config",
        "_amcl.yaml"
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

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        namespace=LaunchConfiguration("robot_name"),
        parameters=[namespaced_config_file, {
            "autostart_node": True,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "initial_pose.x": LaunchConfiguration("x"),
            "initial_pose.y": LaunchConfiguration("y"),
            "initial_pose.yaw": LaunchConfiguration("yaw"),
        }],
        remappings=[("/scan", "scan"), ("map", "/map")]
    )

    return LaunchDescription([
        robot_name_arg,
        use_sim_time_arg,
        initial_pose_x_arg,
        initial_pose_y_arg,
        initial_pose_z_arg,
        amcl,
    ])

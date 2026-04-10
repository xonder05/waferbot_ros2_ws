from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace, SetParameter, SetRemap, LoadComposableNodes, ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterFile, ComposableNode

from nav2_common.launch import RewrittenYaml

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")
    robot_name_arg = DeclareLaunchArgument("robot_name", default_value="waferbot")

    config_file_path = PathJoinSubstitution([
        FindPackageShare("waferbot_navigation_bringup"),
        "config",
        "_navigation.yaml"
    ])

    param_substitutions = {
        "bt_navigator.ros__parameters.robot_base_frame": [LaunchConfiguration("robot_name"), "/base_link"],
        
        "global_costmap.global_costmap.ros__parameters.robot_base_frame": [LaunchConfiguration("robot_name"), "/base_link"],
        
        "local_costmap.local_costmap.ros__parameters.global_frame": [LaunchConfiguration("robot_name"), "/odom"],
        "local_costmap.local_costmap.ros__parameters.robot_base_frame": [LaunchConfiguration("robot_name"), "/base_link"],
        "local_costmap.local_costmap.ros__parameters.voxel_layer.scan.topic": ["/", LaunchConfiguration("robot_name"), "/scan"],

        "collision_monitor.ros__parameters.base_frame_id": [LaunchConfiguration("robot_name"), "/base_footprint"],
        "collision_monitor.ros__parameters.odom_frame_id": [LaunchConfiguration("robot_name"), "/odom"],
        
        "behavior_server.ros__parameters.local_frame": [LaunchConfiguration("robot_name"), "/odom"],
        "behavior_server.ros__parameters.robot_base_frame": [LaunchConfiguration("robot_name"), "/base_link"],

        "docking_server.ros__parameters.base_frame": [LaunchConfiguration("robot_name"), "/base_link"],
        "docking_server.ros__parameters.fixed_frame": [LaunchConfiguration("robot_name"), "/odom"],
    }

    namespaced_config_file = ParameterFile(
        RewrittenYaml(
            source_file=config_file_path,
            root_key=LaunchConfiguration("robot_name"),
            param_rewrites=param_substitutions,
        ),
        allow_substs=True
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    lifecycle_nodes = [
        "bt_navigator",
        "planner_server",
        "smoother_server",
        "controller_server",
        "velocity_smoother",
        "collision_monitor",
        "behavior_server",
        "docking_server",
    ]

    container = ComposableNodeContainer(
        package="rclcpp_components",
        executable="component_container_mt",
        namespace=LaunchConfiguration("robot_name"),
        name="nav2_container",
        parameters=[namespaced_config_file],
    )

    navigation = GroupAction([

        PushRosNamespace(LaunchConfiguration("robot_name")),
        SetParameter("use_sim_time", LaunchConfiguration("use_sim_time")),
        SetParameter("bond_heartbeat_period", 1.0),
        SetRemap("/scan", "scan"),
        SetRemap("map", "/map"),

        LoadComposableNodes(
            target_container=(LaunchConfiguration("robot_name"), "/nav2_container"),
            
            composable_node_descriptions = [

                ComposableNode(
                    package="nav2_bt_navigator",
                    plugin="nav2_bt_navigator::BtNavigator",
                    name="bt_navigator",
                    parameters=[namespaced_config_file],
                ),

                ComposableNode(
                    package="nav2_planner",
                    plugin="nav2_planner::PlannerServer",
                    name="planner_server",
                    parameters=[namespaced_config_file],
                ),

                ComposableNode(
                    package="nav2_smoother",
                    plugin="nav2_smoother::SmootherServer",
                    name="smoother_server",
                    parameters=[namespaced_config_file],
                ),

                ComposableNode(
                    package="nav2_controller",
                    plugin="nav2_controller::ControllerServer",
                    name="controller_server",
                    parameters=[namespaced_config_file],
                    remappings=[("cmd_vel", "cmd_vel_nav")],
                ),

                ComposableNode(
                    package="nav2_velocity_smoother",
                    plugin="nav2_velocity_smoother::VelocitySmoother",
                    name="velocity_smoother",
                    parameters=[namespaced_config_file],
                    remappings=[("cmd_vel", "cmd_vel_nav")],
                ),

                ComposableNode(
                    package="nav2_collision_monitor",
                    plugin="nav2_collision_monitor::CollisionMonitor",
                    name="collision_monitor",
                    parameters=[namespaced_config_file],
                ),

                ComposableNode(
                    package="nav2_behaviors",
                    plugin="behavior_server::BehaviorServer",
                    name="behavior_server",
                    parameters=[namespaced_config_file],
                    remappings=[("cmd_vel", "cmd_vel_nav")],
                ),

                ComposableNode(
                    package="opennav_docking",
                    plugin="opennav_docking::DockingServer",
                    name="docking_server",
                    parameters=[namespaced_config_file],
                    remappings=[("cmd_vel", "diff_drive_controller/cmd_vel")],
                ),

                ComposableNode(
                    package="nav2_lifecycle_manager",
                    plugin="nav2_lifecycle_manager::LifecycleManager",
                    name="lifecycle_manager_navigation",
                    parameters=[
                        {"autostart": True, "node_names": lifecycle_nodes}
                    ],
                ),
            ]
        )
    ])

    return LaunchDescription([
        use_sim_time_arg,
        robot_name_arg,
        stdout_linebuf_envvar,
        container,
        navigation
    ])

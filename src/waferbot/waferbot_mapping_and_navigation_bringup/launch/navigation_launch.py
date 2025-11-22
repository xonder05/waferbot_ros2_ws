from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node, PushRosNamespace, SetParameter, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterFile

from nav2_common.launch import RewrittenYaml

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    robot_name_arg = DeclareLaunchArgument('robot_name', default_value="waferbot")

    config_file_path = PathJoinSubstitution([
        FindPackageShare('waferbot_mapping_and_navigation_bringup'),
        'config',
        'nav2_navigation.yaml'
    ])

    param_substitutions = {
        'bt_navigator.ros__parameters.robot_base_frame': [LaunchConfiguration('robot_name'), '/base_link'],
        
        'global_costmap.global_costmap.ros__parameters.robot_base_frame': [LaunchConfiguration('robot_name'), '/base_link'],
        
        'local_costmap.local_costmap.ros__parameters.global_frame': [LaunchConfiguration('robot_name'), '/odom'],
        'local_costmap.local_costmap.ros__parameters.robot_base_frame': [LaunchConfiguration('robot_name'), '/base_link'],

        'collision_monitor.ros__parameters.base_frame_id': [LaunchConfiguration('robot_name'), '/base_footprint'],
        'collision_monitor.ros__parameters.odom_frame_id': [LaunchConfiguration('robot_name'), '/odom'],
        
        'behavior_server.ros__parameters.local_frame': [LaunchConfiguration('robot_name'), '/odom'],
        'behavior_server.ros__parameters.robot_base_frame': [LaunchConfiguration('robot_name'), '/base_link'],
    }

    namespaced_config_file = ParameterFile(
        RewrittenYaml(
            source_file=config_file_path,
            root_key=LaunchConfiguration('robot_name'),
            param_rewrites=param_substitutions,
        ),
        allow_substs=True
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    lifecycle_nodes = [
        'bt_navigator',
        'planner_server',
        'smoother_server',
        'controller_server',
        'velocity_smoother',
        'collision_monitor',
        'behavior_server',
    ]

    navigation = GroupAction([
        PushRosNamespace(LaunchConfiguration('robot_name')),
        SetParameter('use_sim_time', LaunchConfiguration("use_sim_time")),
        SetRemap('/scan', 'scan'),
        SetRemap('map', '/map'),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            parameters=[{'autostart': True}, {'node_names': lifecycle_nodes}],
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            parameters=[namespaced_config_file],
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            parameters=[namespaced_config_file],
        ),

        Node(
            package='nav2_smoother',
            executable='smoother_server',
            parameters=[namespaced_config_file],
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            parameters=[namespaced_config_file],
            remappings=[('cmd_vel', 'cmd_vel_nav')],
        ),

        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            parameters=[namespaced_config_file],
            remappings=[('cmd_vel', 'cmd_vel_nav')],
        ),

        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            parameters=[namespaced_config_file],
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            parameters=[namespaced_config_file],
            remappings=[('cmd_vel', 'cmd_vel_nav')],
        )
    ])

    return LaunchDescription([
        use_sim_time_arg,
        robot_name_arg,

        stdout_linebuf_envvar,
        navigation
    ])
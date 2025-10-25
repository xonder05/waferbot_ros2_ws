from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterFile

from nav2_common.launch import RewrittenYaml

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='waferbot')

    config_file_path = PathJoinSubstitution([
        FindPackageShare('waferbot_mapping_and_navigation_bringup'),
        'config',
        'nav2_localization.yaml'
    ])

    param_substitutions = {
        'amcl.ros__parameters.base_frame_id': [LaunchConfiguration('robot_name'), '/base_footprint'],
        'amcl.ros__parameters.odom_frame_id': [LaunchConfiguration('robot_name'), '/odom'],
    }

    namespaced_config_file = ParameterFile(
        RewrittenYaml(
            source_file=config_file_path,
            root_key=LaunchConfiguration('robot_name'),
            param_rewrites=param_substitutions,
        ),
        allow_substs=True
    )

    localization = Node(
        package='nav2_amcl',
        executable='amcl',
        namespace=LaunchConfiguration('robot_name'),
        parameters=[namespaced_config_file, {
            "autostart_node": True,
            'use_sim_time': LaunchConfiguration("use_sim_time")
        }],
        remappings=[('map', '/map'), ('/scan', 'scan')]
    )


    return LaunchDescription([
        use_sim_time_arg,
        robot_name_arg,

        localization
    ])

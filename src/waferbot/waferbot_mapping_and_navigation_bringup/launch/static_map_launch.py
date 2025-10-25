from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    
    map_file_arg = DeclareLaunchArgument('map_file', default_value='')

    config_file_path = PathJoinSubstitution([
        FindPackageShare('waferbot_mapping_and_navigation_bringup'),
        'config',
        'nav2_map_server.yaml'
    ])

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[config_file_path, {
            "autostart_node": True, 
            "use_sim_time": LaunchConfiguration("use_sim_time"), 
            'yaml_filename': LaunchConfiguration("map_file")
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_file_arg,

        map_server
    ])

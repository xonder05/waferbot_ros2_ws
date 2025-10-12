from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
    )
    
    config_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            get_package_share_directory('waferbot_mapping_and_navigation_bringup'),
            'config',
            'nav2_params.yaml'
        )
    )

    # ros2 launch nav2_bringup navigation_launch.py params_file:=./src/mapping_and_navigation/config/nav2_params.yaml 
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("nav2_bringup"), '/launch', '/navigation_launch.py'
        ]),
        launch_arguments={
            "use_sim_time": LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('slam_params_file')
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,

        nav2
    ])
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
            get_package_share_directory('mapping_and_navigation'),
            'config',
            'mapper_params_online_async.yaml'
        )
    )

    #ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/mapping_and_navigation/config/mapper_params_online_async.yaml 
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("slam_toolbox"), '/launch', '/online_async_launch.py'
        ]),
        launch_arguments={
            "use_sim_time": LaunchConfiguration('use_sim_time'),
            'slam_params_file': LaunchConfiguration('slam_params_file')
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,

        slam_toolbox
    ])
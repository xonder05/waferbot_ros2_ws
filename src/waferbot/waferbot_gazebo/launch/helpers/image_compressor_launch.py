import os, launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value="waferbot")

    config_file = os.path.join(
        get_package_share_directory('waferbot_gazebo'),
        'config',
        'helpers_config.yaml'
    )

    image_compressor = Node(
        package='waferbot_gazebo',
        executable='image_compressor',
        namespace=LaunchConfiguration("robot_name"),
        parameters=[config_file]
    )
    
    return launch.LaunchDescription([
        robot_name_arg,
        
        image_compressor,
    ])

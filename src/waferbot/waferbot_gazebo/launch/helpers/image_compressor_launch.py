import os, launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    config_file = os.path.join(
        get_package_share_directory('waferbot_gazebo'),
        'config',
        'helpers_config.yaml'
    )

    image_compressor = Node(
        package='waferbot_gazebo',
        executable='image_compressor',
        parameters=[config_file]
    )
    
    return launch.LaunchDescription([
        image_compressor,
    ])

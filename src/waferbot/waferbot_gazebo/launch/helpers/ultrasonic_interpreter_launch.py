import os, launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    config_file = os.path.join(
        get_package_share_directory('waferbot_gazebo'),
        'config',
        'config.yaml'
    )

    ultrasonic_interpreter = Node(
        package='waferbot_gazebo',
        executable='ultrasonic_interpreter',
        parameters=[config_file]
    )
    
    return launch.LaunchDescription([
        ultrasonic_interpreter,
    ])
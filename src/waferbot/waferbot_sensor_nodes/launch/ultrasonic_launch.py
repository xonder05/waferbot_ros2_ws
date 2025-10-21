import os
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value="waferbot")

    config_file = os.path.join(
        get_package_share_directory('waferbot_sensor_nodes'),
        'config',
        'config.yaml'
    )

    ultrasonic_node = Node(
        package='waferbot_sensor_nodes',
        executable='ultrasonic_node',
        namespace=LaunchConfiguration("robot_name"),
        parameters=[config_file]
    )
    
    return launch.LaunchDescription([
        robot_name_arg,

        ultrasonic_node,
    ])

import os, launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value="waferbot")

    config_file = os.path.join(
        get_package_share_directory('waferbot_gazebo'),
        'config',
        'helpers_config.yaml'
    )

    ultrasonic_interpreter = Node(
        package='waferbot_gazebo',
        executable='ultrasonic_interpreter',
        namespace=LaunchConfiguration("robot_name"),
        parameters=[config_file]
    )
    
    return launch.LaunchDescription([
        robot_name_arg,

        ultrasonic_interpreter,
    ])
import os
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value="waferbot")

    config_file = os.path.join(
        get_package_share_directory('waferbot_gazebo'),
        'config',
        'bridges_config.yaml'
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace=LaunchConfiguration("robot_name"),
        parameters=[{
            "expand_gz_topic_names": True,
            "config_file": config_file
        }]
    )

    return launch.LaunchDescription([
        robot_name_arg,

        ros_gz_bridge,
    ])

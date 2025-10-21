from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    use_sim_arg = DeclareLaunchArgument('use_sim_time', default_value="false")
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value="waferbot")

    xacro_model_path = os.path.join(
        get_package_share_directory('waferbot_description'),
        'description',
        'waferbot.xacro'
    )

    urdf_model = Command(['xacro ', str(xacro_model_path), ' use_sim:=', LaunchConfiguration("use_sim_time"), " robot_name:=", LaunchConfiguration("robot_name")])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=LaunchConfiguration("robot_name"),
        parameters=[
            {"frame_prefix": [LaunchConfiguration("robot_name"), TextSubstitution(text="/")]},
            {'robot_description': urdf_model},
            {'use_sim_time': LaunchConfiguration("use_sim_time")},
        ],
    )

    return LaunchDescription([
        use_sim_arg,
        robot_name_arg,

        robot_state_publisher,
    ])
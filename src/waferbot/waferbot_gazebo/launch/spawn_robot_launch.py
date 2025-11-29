from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    
    robot_name_arg = DeclareLaunchArgument("robot_name", default_value="waferbot")
    spawn_x_arg = DeclareLaunchArgument("x", default_value="0.0")
    spawn_y_arg = DeclareLaunchArgument("y", default_value="0.0")
    spawn_z_arg = DeclareLaunchArgument("z", default_value="0.5")

    spawn_robot = Node(
        package="ros_gz_sim", 
        executable="create",
        parameters=[{
            "topic": [LaunchConfiguration("robot_name"), "/robot_description"],
            "x": LaunchConfiguration("x"),
            "y": LaunchConfiguration("y"),
            "z": LaunchConfiguration("z"),
        }],
    )

    return LaunchDescription([
        robot_name_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        spawn_robot,
    ])

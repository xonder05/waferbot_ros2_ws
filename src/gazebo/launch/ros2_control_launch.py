import launch
from launch_ros.actions import Node

def generate_launch_description():

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager-timeout", "60"],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "60"],
    )

    return launch.LaunchDescription([
        diff_drive_controller_spawner,
        joint_state_broadcaster_spawner,
    ])
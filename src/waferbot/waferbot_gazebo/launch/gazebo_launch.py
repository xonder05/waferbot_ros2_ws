from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    world_select_arg = DeclareLaunchArgument('world_select', default_value='wandering')

    world_file_path = PathJoinSubstitution([
        FindPackageShare("waferbot_gazebo"),
        "worlds",
        [LaunchConfiguration('world_select'), TextSubstitution(text="_world.sdf")]
    ])

    gz_args = [TextSubstitution(text="-v 4 "), TextSubstitution(text="-r "), world_file_path]

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ros_gz_sim'), '/launch', '/gz_sim.launch.py'
        ]),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    #ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock
    clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]
    )

    return LaunchDescription([
        world_select_arg,
        
        simulator,
        clock,
    ])

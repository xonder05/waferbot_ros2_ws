from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    config_select_arg = DeclareLaunchArgument('config_file', default_value='single_robot.rviz')

    rviz_config_path = PathJoinSubstitution([
        FindPackageShare("waferbot_description"),
        'config',
        LaunchConfiguration("config_file")
    ])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        config_select_arg,
        
        rviz,
    ])
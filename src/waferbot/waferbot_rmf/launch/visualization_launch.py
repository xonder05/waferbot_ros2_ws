from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")

    config_file_path = PathJoinSubstitution([
        FindPackageShare("waferbot_rmf"),
        "config",
        "_visualization_config.yaml"
    ])

    navgraph = Node(
        package="rmf_visualization_navgraphs",
        executable="navgraph_visualizer_node",
        parameters=[config_file_path, {
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )

    schedule = Node(
        package="rmf_visualization_schedule",
        executable="schedule_visualizer_node",
        parameters=[config_file_path, {
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )

    fixer = Node(
        package="waferbot_rmf",
        executable="rmf_visualization_fixer.py",
        parameters=[config_file_path, {
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        navgraph,
        schedule,
        fixer
    ])

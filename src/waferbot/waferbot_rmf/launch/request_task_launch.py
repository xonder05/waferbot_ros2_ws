from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    fleet_arg = DeclareLaunchArgument("fleet", default_value="")
    robot_arg = DeclareLaunchArgument("robot", default_value="")
    start_time_arg = DeclareLaunchArgument("start_time", default_value="0")
    task_arg = DeclareLaunchArgument("task", default_value="")
    place_arg = DeclareLaunchArgument("place", default_value="")
    places_arg = DeclareLaunchArgument("places", default_value="['', '']")
    rounds_arg = DeclareLaunchArgument("rounds", default_value="0")
    task_id_arg = DeclareLaunchArgument("task_id", default_value="")

    task_generator = Node(
        package="waferbot_rmf",
        executable="task_requester.py",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "fleet": LaunchConfiguration("fleet"),
            "robot": LaunchConfiguration("robot"),
            "start_time": LaunchConfiguration("start_time"),
            "task": LaunchConfiguration("task"),
            "place": LaunchConfiguration("place"),
            "places": LaunchConfiguration("places"),
            "rounds": LaunchConfiguration("rounds"),
            "task_id": LaunchConfiguration("task_id"),
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        fleet_arg,
        robot_arg,
        start_time_arg,
        task_arg,
        place_arg,
        places_arg,
        rounds_arg,
        task_id_arg,
        task_generator
    ])

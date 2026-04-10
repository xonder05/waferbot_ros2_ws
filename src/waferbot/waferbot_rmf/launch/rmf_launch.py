from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    map_name_arg = DeclareLaunchArgument("map", default_value="warehouse")

    config_file_path = PathJoinSubstitution([
        FindPackageShare("waferbot_rmf"),
        "config",
        "waferbot_adapter_config.yaml"
    ])

    building_map_file = PathJoinSubstitution([
        FindPackageShare("waferbot_rmf"),
        "maps", 
        [LaunchConfiguration("map"), ".building.yaml"]
    ])

    nav_graph_file = PathJoinSubstitution([
        FindPackageShare("waferbot_rmf"),
        "maps", 
        LaunchConfiguration("map"),
        "nav_graphs/0.yaml",
    ])

    building_map = Node(
        package="rmf_building_map_tools",
        executable="building_map_server",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }],
        arguments=[
            building_map_file
        ],
        remappings=[
            ("map", "building_map")
        ]
    )

    dispatcher = Node(
        package="rmf_task_ros2",
        executable="rmf_task_dispatcher",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )

    traffic_schedule = Node(
        package="rmf_traffic_ros2",
        executable="rmf_traffic_schedule",
        name="rmf_traffic_schedule_primary",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }],
    )
    
    blockade_moderator = Node(
        package="rmf_traffic_ros2",
        executable="rmf_traffic_blockade",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }],
    )

    mutex_group_supervisor = Node(
        package="rmf_fleet_adapter",
        executable="mutex_group_supervisor",
        parameters=[],
    )

    fleet_adapter = Node(
        package="waferbot_rmf",
        executable="fleet_adapter.py",
        parameters=[],
        arguments=[
            "-sim", 
            "-c", config_file_path,
            "-n", nav_graph_file, 
        ],
    )

    mock_dis_ing = Node(
        package="waferbot_rmf",
        executable="mock_dispenser_ingestor.py",
        parameters=[],
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_name_arg,
        building_map,
        dispatcher,
        traffic_schedule,
        blockade_moderator,
        mutex_group_supervisor,
        fleet_adapter,
        mock_dis_ing,
    ])

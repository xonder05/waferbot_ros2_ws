from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import FrontendLaunchDescriptionSource

from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")

    config_file_path = PathJoinSubstitution([
        FindPackageShare("waferbot_rmf"),
        "config",
        "waferbot_adapter_config.yaml"
    ])

    nav_graph_file = PathJoinSubstitution([
        FindPackageShare("waferbot_rmf"),
        "maps/warehouse/nav_graphs/0.yaml",
    ])

    viz_config_file = PathJoinSubstitution([
        FindPackageShare("rmf_visualization_schedule"),
        "config/rmf.rviz",
    ])

    # common
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

    building_map = Node(
        package="rmf_building_map_tools",
        executable="building_map_server",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }],
        arguments=[
            "/home/daniel/ros2_ws/src/waferbot/waferbot_rmf/maps/warehouse.building.yaml"
        ],
        remappings=[
            ("map", "building_map")
        ]
    )

    dispatcher = Node(
        package="rmf_task_ros2",
        executable="rmf_task_dispatcher",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }],
        # not needed right now but fix it later
        # <param name="use_sim_time" value="$(var use_sim_time)"/>
        # <param name="bidding_time_window" value="$(var bidding_time_window)"/>
        # <param name="use_unique_hex_string_with_task_id" value="$(var use_unique_hex_string_with_task_id)"/>
        # <param name="server_uri" value="$(var server_uri)"/>
    )

    fleet_adapter = Node(
        package="waferbot_rmf",
        executable="fleet_adapter.py",
        parameters=[{}],
        arguments=[
            "-sim", 
            "-n", nav_graph_file, 
            "-c", config_file_path
            # "server_uri" : "",
        ],
        output="screen"
    )

    visualization = GroupAction(
        actions=[
            SetRemap("/map", "/building_map"),

            IncludeLaunchDescription(
                FrontendLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("rmf_visualization"), 
                        "visualization.launch.xml",
                    ])
                ),
                launch_arguments=[
                    ("use_sim_time", LaunchConfiguration("use_sim_time")),           
                    ("map_name", "L1"),
                    ("viz_config_file", viz_config_file),
                    ("headless", "true"),
                    ("rmf_frame_id", "rmf_map")      
                ],
            )
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,

        traffic_schedule,
        dispatcher,
        blockade_moderator,
        building_map,
        fleet_adapter,
        visualization
    ])

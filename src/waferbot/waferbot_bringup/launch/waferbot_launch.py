from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

from launch_ros.substitutions import FindPackageShare

def keyword_condition(keyword):
    return IfCondition(
        PythonExpression([
            f"'{keyword}' in '", LaunchConfiguration("targets"), "'"
        ])
    )

def validate_args(context, *args, **kwargs):

    targets_arg_string = LaunchConfiguration("targets").perform(context)
    targets = list(targets_arg_string.lower().split())

    for target in targets:
        if target not in ["rsp", "real", "sim", "localization", "wandering", "live_mapping", "static_map", "navigation"]:
            raise RuntimeError(f"One of the targets you provided is {target}, which is not a valid target, please check your spelling or list of valid targets.")

    if "sim" in targets and "real" in targets:
        raise RuntimeError("Running real robot and simulator from single launch file will lead to wrong namespacing and duplicate data in topics, please pick either real or sim, not both.")

    if "wandering" in targets and "navigation" in targets:
        raise RuntimeError("Both wandering and navigation handle movement of the robot, please pick just one of them at a time.")

def generate_launch_description():

    robot_name_arg = DeclareLaunchArgument("robot_name", default_value="waferbot")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")
    targets_arg = DeclareLaunchArgument("targets", default_value="real", 
        description="Space separated list of launch targets. Allowed values are: real, sim, localization, live_mapping, static_map, navigation"
    )

    validate_args_ = OpaqueFunction(function=validate_args)

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("waferbot_description"), 
                "launch", 
                "robot_state_publisher_launch.py"
            ])
        ),
        launch_arguments=[
            ("robot_name", LaunchConfiguration("robot_name")),
            ("use_sim_time", LaunchConfiguration("use_sim_time"))
        ],
        condition=keyword_condition("rsp")
    )

    real_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("waferbot_bringup"), 
                "launch", 
                "real_robot_launch.py"
            ])
        ),
        launch_arguments=[
            ("robot_name", LaunchConfiguration("robot_name"))            
        ],
        condition=keyword_condition("real")
    )

    sim_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("waferbot_bringup"), 
                "launch", 
                "simulation_launch.py"
            ])
        ),
        launch_arguments=[
            ("robot_name", LaunchConfiguration("robot_name"))            
        ],
        condition=keyword_condition("sim")
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("waferbot_localization_bringup"), 
                "launch", 
                "ekf_fusion_launch.py"
            ])
        ),
        launch_arguments=[
            ("robot_name", LaunchConfiguration("robot_name")),
            ("use_sim_time", LaunchConfiguration("use_sim_time"))
        ],
        condition=keyword_condition("localization")
    )

    motion_executor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("waferbot_behaviors"), 
                "launch", 
                "motion_executor_launch.py"
            ])
        ),
        launch_arguments=[
            ("robot_name", LaunchConfiguration("robot_name")),
            ("use_sim_time", LaunchConfiguration("use_sim_time"))
        ],
        condition=keyword_condition("wandering")
    )

    wandering = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("waferbot_behaviors"), 
                "launch", 
                "wandering_launch.py"
            ])
        ),
        launch_arguments=[
            ("robot_name", LaunchConfiguration("robot_name")),
            ("use_sim_time", LaunchConfiguration("use_sim_time"))
        ],
        condition=keyword_condition("wandering")
    )

    live_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("waferbot_mapping_bringup"), 
                "launch", 
                "live_mapping_launch.py"
            ])
        ),
        launch_arguments=[
            ("robot_name", LaunchConfiguration("robot_name")),
            ("use_sim_time", LaunchConfiguration("use_sim_time"))
        ],
        condition=keyword_condition("live_mapping")
    )

    static_map = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("waferbot_mapping_bringup"), 
                "launch", 
                "static_map_launch.py"
            ])
        ),
        launch_arguments=[
            ("robot_name", LaunchConfiguration("robot_name")),
            ("use_sim_time", LaunchConfiguration("use_sim_time")),
        ],
        condition=keyword_condition("static_map")
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("waferbot_navigation_bringup"), 
                "launch", 
                "navigation_launch.py"
            ])
        ),
        launch_arguments=[
            ("robot_name", LaunchConfiguration("robot_name")),
            ("use_sim_time", LaunchConfiguration("use_sim_time"))
        ],
        condition=keyword_condition("navigation")
    )

    return LaunchDescription([
        robot_name_arg,
        use_sim_time_arg,
        targets_arg,
        validate_args_,

        robot_state_publisher,
        real_robot,
        sim_robot,
        localization,
        motion_executor,
        wandering,
        static_map,
        live_mapping,
        navigation
    ])

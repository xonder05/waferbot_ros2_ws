import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

from launch_ros.substitutions import FindPackageShare

def parse_targets():
    
    all_targets_array = [
        # preset, included goals
        "globals", "gazebo", "map", "rmf",
        "controllers", "ddc", "fcc", "jsp",
        "sensors", "imu", "lidar", "ultra", "camera",
        "real", "rsp", "conman", # controllers sensors
        "sim", "spawn", "bridges", # rsp, controllers
        "wandering", "helpers",
        # not in any preset
        "loc",  "slam", "amcl", "nav"
    ]

    targets_string = next((arg[9:] for arg in sys.argv if arg[:7] == "targets"), "rsp")
    targets_array = list(targets_string.lower().split())

    for target in targets_array:
        if target not in all_targets_array:
            raise RuntimeError(f"One of the targets you provided is {target}, which is not a valid target, please check your spelling or list of valid targets.")

    if "sim" in targets_array and "real" in targets_array:
        raise RuntimeError("Running real robot and simulator from single launch file will lead to wrong namespacing and duplicate data in topics, please pick either real or sim, not both.")

    if "wandering" in targets_array and "navigation" in targets_array:
        raise RuntimeError("Both wandering and navigation handle movement of the robot, please pick just one of them at a time.")

    launch_targets = {}
    launch_targets["gazebo"] = True if any(t in targets_array for t in ["gazebo", "globals"]) else False
    launch_targets["map"] = True if any(t in targets_array for t in ["map", "globals"]) else False
    launch_targets["rmf"] = True if any(t in targets_array for t in ["rmf", "globals"]) else False
    launch_targets["rsp"] = True if any(t in targets_array for t in ["rsp", "real", "sim"]) else False
    launch_targets["spawn"] = True if any(t in targets_array for t in ["spawn", "sim"]) else False
    launch_targets["bridges"] = True if any(t in targets_array for t in ["bridges", "sim"]) else False
    launch_targets["helpers"] = True if any(t in targets_array for t in ["helpers", "wandering"]) else False
    launch_targets["conman"] = True if any(t in targets_array for t in ["conman", "real"]) else False
    launch_targets["ddc"] = True if any(t in targets_array for t in ["ddc", "controllers", "real", "sim"]) else False
    launch_targets["fcc"] = True if any(t in targets_array for t in ["fcc", "controllers", "real", "sim"]) else False
    launch_targets["jsp"] = True if any(t in targets_array for t in ["jsp", "controllers", "real", "sim"]) else False
    launch_targets["imu"] = True if any(t in targets_array for t in ["imu", "sensors", "real"]) else False
    launch_targets["lidar"] = True if any(t in targets_array for t in ["lidar", "sensors", "real"]) else False
    launch_targets["ultra"] = True if any(t in targets_array for t in ["ultra", "sensors", "real"]) else False
    launch_targets["camera"] = True if any(t in targets_array for t in ["camera", "sensors", "real"]) else False
    launch_targets["loc"] = True if any(t in targets_array for t in ["loc"]) else False
    launch_targets["wandering"] = True if any(t in targets_array for t in ["wandering"]) else False
    launch_targets["slam"] = True if any(t in targets_array for t in ["slam"]) else False
    launch_targets["amcl"] = True if any(t in targets_array for t in ["amcl"]) else False
    launch_targets["nav"] = True if any(t in targets_array for t in ["nav"]) else False
    
    return launch_targets

def generate_launch_description():

    robot_name_arg = DeclareLaunchArgument("robot_name", default_value="waferbot")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")
    world_select_arg = DeclareLaunchArgument("world_select", default_value="warehouse")
    x_robot_position_arg = DeclareLaunchArgument("x", default_value="0.0")
    y_robot_position_arg = DeclareLaunchArgument("y", default_value="0.0")
    z_robot_position_arg = DeclareLaunchArgument("z", default_value="0.0")
    yaw_robot_rotation_arg = DeclareLaunchArgument("yaw", default_value="0.0")
    enable_odom_tf_arg = DeclareLaunchArgument("enable_odom_tf", default_value="True")
    optimize_sim_perf_arg = DeclareLaunchArgument("optimize_sim_perf", default_value="True")

    targets_arg = DeclareLaunchArgument("targets", default_value="real", 
        description="Space separated list of launch targets. Allowed values are: real, sim, localization, live_mapping, static_map, navigation"
    )
    launch_targets = parse_targets()


    # ---------- Robot Independent ----------

    # simulator and world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("waferbot_gazebo"), 
                "launch", 
                "gazebo_launch.py"
            ])
        ),
        launch_arguments=[
            ("world_select", LaunchConfiguration("world_select"))
        ],
        condition=IfCondition(str(launch_targets["gazebo"]))
    )

    map_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("waferbot_mapping_bringup"), 
                "launch", 
                "map_server_launch.py"
            ])
        ),
        launch_arguments=[
            ("robot_name", LaunchConfiguration("robot_name")),
            ("use_sim_time", LaunchConfiguration("use_sim_time")),
            ("map_select", LaunchConfiguration("world_select"))
        ],
        condition=IfCondition(str(launch_targets["map"]))
    )

    rmf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("waferbot_rmf"), 
                "launch",
                "launch.py"
            ])
        ),
        condition=IfCondition(str(launch_targets["rmf"]))
    )

    # ---------- Robot Specific ----------

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
            ("use_sim_time", LaunchConfiguration("use_sim_time")),
            ("optimize_sim_perf", LaunchConfiguration("optimize_sim_perf"))
        ],
        condition=IfCondition(str(launch_targets["rsp"]))
    )

    # robot from description
    robot_spawn = TimerAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("waferbot_gazebo"), 
                        "launch", 
                        "spawn_robot_launch.py"
                    ])
                ),
                launch_arguments=[
                    ("robot_name", LaunchConfiguration("robot_name")),
                    ("x", LaunchConfiguration("x")),
                    ("y", LaunchConfiguration("y")),
                    ("z", LaunchConfiguration("z"))
                ]
            )
        ],
        period=5.0,
        condition=IfCondition(str(launch_targets["spawn"]))
    )

    # ---------- Sensors ----------

    # ros_gz_bridges
    bridges = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("waferbot_gazebo"), 
                "launch", 
                "bridges_launch.py"
            ])
        ),
        launch_arguments=[
            ("robot_name", LaunchConfiguration("robot_name"))
        ],
        condition=IfCondition(str(launch_targets["bridges"]))
    )

    # helper nodes
    helpers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("waferbot_gazebo"), 
                "launch", 
                "helpers_launch.py"
            ])
        ),
        launch_arguments=[
            ("robot_name", LaunchConfiguration("robot_name"))
        ],
        condition=IfCondition(str(launch_targets["helpers"]))
    )

    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("waferbot_sensors"),
                "launch", 
                "imu_launch.py"
            ])
        ),
        condition=IfCondition(str(launch_targets["imu"]))
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("waferbot_sensors"),
                "launch", 
                "ydlidar_launch.py"
            ])
        ),
        condition=IfCondition(str(launch_targets["lidar"]))
    )

    ultrasonic = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("waferbot_sensors"),
                "launch", 
                "ultrasonic_launch.py"
            ])
        ),
        condition=IfCondition(str(launch_targets["ultra"]))
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("waferbot_sensors"),
                "launch", 
                "camera_launch.py"
            ])
        ),
        condition=IfCondition(str(launch_targets["camera"]))
    )

    # ---------- Actuators ----------

    controller_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("waferbot_control_bringup"), 
                "launch", 
                "controller_manager_launch.py"
            ])
        ),
        launch_arguments=[
            ("robot_name", LaunchConfiguration("robot_name")),
            ("use_sim_time", LaunchConfiguration("use_sim_time"))
        ],
        condition=IfCondition(str(launch_targets["conman"]))
    )

    diff_drive_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("waferbot_control_bringup"), 
                "launch", 
                "diff_drive_controller_launch.py"
            ])
        ),
        launch_arguments=[
            ("robot_name", LaunchConfiguration("robot_name")),
            ("enable_odom_tf", LaunchConfiguration("enable_odom_tf"))
        ],
        condition=IfCondition(str(launch_targets["ddc"]))
    )

    forward_command_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("waferbot_control_bringup"), 
                "launch", 
                "forward_command_controller_launch.py"
            ])
        ),
        launch_arguments=[
            ("robot_name", LaunchConfiguration("robot_name"))
        ],
        condition=IfCondition(str(launch_targets["fcc"]))
    )

    joint_state_broadcaster = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("waferbot_control_bringup"), 
                "launch", 
                "joint_state_broadcaster_launch.py"
            ])
        ),
        launch_arguments=[
            ("robot_name", LaunchConfiguration("robot_name"))
        ],
        condition=IfCondition(str(launch_targets["jsp"]))
    )

    # ---------- Cognitive ----------

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
        condition=IfCondition(str(launch_targets["loc"]))
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
        condition=IfCondition(str(launch_targets["wandering"]))
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
        condition=IfCondition(str(launch_targets["wandering"]))
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("waferbot_mapping_bringup"), 
                "launch", 
                "slam_launch.py"
            ])
        ),
        launch_arguments=[
            ("robot_name", LaunchConfiguration("robot_name")),
            ("use_sim_time", LaunchConfiguration("use_sim_time"))
        ],
        condition=IfCondition(str(launch_targets["slam"]))
    )

    amcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("waferbot_mapping_bringup"), 
                "launch", 
                "amcl_launch.py"
            ])
        ),
        launch_arguments=[
            ("robot_name", LaunchConfiguration("robot_name")),
            ("use_sim_time", LaunchConfiguration("use_sim_time")),
            ("x", LaunchConfiguration("x")),
            ("y", LaunchConfiguration("y")),
            ("yaw", LaunchConfiguration("yaw"))
        ],
        condition=IfCondition(str(launch_targets["amcl"]))
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("waferbot_navigation_bringup"), 
                "launch", 
                "composable_navigation_launch.py"
            ])
        ),
        launch_arguments=[
            ("robot_name", LaunchConfiguration("robot_name")),
            ("use_sim_time", LaunchConfiguration("use_sim_time"))
        ],
        condition=IfCondition(str(launch_targets["nav"]))
    )

    return LaunchDescription([
        robot_name_arg,
        use_sim_time_arg,
        world_select_arg,
        x_robot_position_arg,
        y_robot_position_arg,
        z_robot_position_arg,
        yaw_robot_rotation_arg,
        enable_odom_tf_arg,
        optimize_sim_perf_arg,
        targets_arg,

        gazebo,
        map_server,
        rmf,
        robot_state_publisher,
        robot_spawn,
        bridges,
        helpers,
        imu,
        lidar,
        ultrasonic,
        camera,
        controller_manager,
        diff_drive_controller,
        forward_command_controller,
        joint_state_broadcaster,
        localization,
        motion_executor,
        wandering,
        slam,
        amcl,
        navigation
    ])

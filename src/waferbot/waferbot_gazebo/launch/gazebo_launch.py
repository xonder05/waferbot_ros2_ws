from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

from ros_gz_bridge.actions import RosGzBridge

def generate_launch_description():
    
    world_select_arg = DeclareLaunchArgument("world_select", default_value="wandering")

    config_file_path = PathJoinSubstitution([
        FindPackageShare("waferbot_gazebo"),
        "config",
        "_clock_bridge.yaml"
    ])

    world_file_path = PathJoinSubstitution([
        FindPackageShare("waferbot_gazebo"),
        "worlds",
        LaunchConfiguration("world_select"),
        [LaunchConfiguration("world_select"), "_world.sdf"]
    ])

    gz_env = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=FindPackageShare("waferbot_gazebo")
    )

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"), 
                "launch", 
                "gz_sim.launch.py"
            ])
        ),
        launch_arguments=[
            ("gz_args", ["-v 4 -r ", world_file_path])            
        ]
    )

    ros_gz_bridge_container = RosGzBridge (
        
        # create container (robot bridge nodes will reuse this)
        use_composition=True,
        container_name="ros_gz_bridge_container",
        create_own_container=True,
        
        # start clock bridge
        bridge_name="ros_gz_bridge",
        config_file=config_file_path,
    )

    return LaunchDescription([
        world_select_arg,
        gz_env,
        simulator,
        ros_gz_bridge_container,
    ])

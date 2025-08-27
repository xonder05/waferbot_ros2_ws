import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    
    world_select_arg = DeclareLaunchArgument('world_select', default_value='wandering')
    world_select_val = LaunchConfiguration('world_select')

    xacro_model_path = os.path.join(
        get_package_share_directory('robot_description'),
        'description',
        'robot.xacro'
    )

    urdf_model_path = os.path.join(
        get_package_share_directory('robot_description'),
        'description',
        'robot.urdf'
    )

    #xacro robot.xacro -o robot.urdf
    convert_xacro = ExecuteProcess(
        cmd=['xacro', xacro_model_path, '-o', urdf_model_path],
        output='screen',
    )

    gazebo_spawner_service = PathJoinSubstitution([
        "world",
        world_select_val,
        "create",
    ])

    #ign service -s /world/empty/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/home/daniel/ros2_ws/src/gazebo_simulator_nodes/models/robot.urdf", name: "urdf_model"'
    spawn_robot = ExecuteProcess(
        cmd=['gz', 'service', 
             '--service', gazebo_spawner_service, 
             '--reqtype', 'gz.msgs.EntityFactory', 
             '--reptype', 'gz.msgs.Boolean', 
             '--timeout', '60000', 
             '--req', f'sdf_filename: "{urdf_model_path}", name: "robot"'],
        output='screen',
    )

    remove_urdf = ExecuteProcess(
        cmd=['rm', urdf_model_path],
        output='screen',
    )

    urdf_creation_event = RegisterEventHandler(
        OnProcessExit(
            target_action=convert_xacro,
            on_exit=[spawn_robot]
        )
    )

    urdf_cleanup_event = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[remove_urdf]
        )
    )

    return LaunchDescription([
        world_select_arg,
        convert_xacro,
        urdf_creation_event,
        urdf_cleanup_event
    ])
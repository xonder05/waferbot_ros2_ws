from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    robot_name_arg = DeclareLaunchArgument("robot_name", default_value="waferbot")
    use_sim_arg = DeclareLaunchArgument("use_sim_time", default_value="False")
    optimize_sim_perf_arg = DeclareLaunchArgument("optimize_sim_perf", default_value="False")

    xacro_model_path = PathJoinSubstitution([
        FindPackageShare("waferbot_description"),
        "xacro",
        "waferbot.xacro"
    ])

    urdf_model = Command([
        "xacro ",
        xacro_model_path,
        " robot_name:=", LaunchConfiguration("robot_name"),
        " use_sim:=", LaunchConfiguration("use_sim_time"),
        " optimize_sim_perf:=", LaunchConfiguration("optimize_sim_perf"),
    ])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=LaunchConfiguration("robot_name"),
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_description": urdf_model,
            "frame_prefix": [LaunchConfiguration("robot_name"), "/"],
        }],
    )

    return LaunchDescription([
        robot_name_arg,
        use_sim_arg,
        optimize_sim_perf_arg,
        robot_state_publisher,
    ])

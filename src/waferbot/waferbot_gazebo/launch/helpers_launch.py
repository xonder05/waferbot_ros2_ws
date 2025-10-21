from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    robot_name_arg = DeclareLaunchArgument('robot_name', default_value="waferbot")

    ultrasonic_interpreter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("waferbot_gazebo"), '/launch/helpers', '/ultrasonic_interpreter_launch.py'
        ]),
        launch_arguments={"robot_name": LaunchConfiguration("robot_name")}.items()  
    )

    image_compressor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("waferbot_gazebo"), '/launch/helpers', '/image_compressor_launch.py'
        ]),
        launch_arguments={"robot_name": LaunchConfiguration("robot_name")}.items()  
    )

    return LaunchDescription([
        robot_name_arg,

        ultrasonic_interpreter,
        image_compressor,
    ])
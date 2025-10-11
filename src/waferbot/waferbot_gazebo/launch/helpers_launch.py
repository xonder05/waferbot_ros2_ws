from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # servo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         FindPackageShare("waferbot_gazebo"), '/launch/bridges', '/simulator_servo_launch.py'
    #     ])
    # )

    ultrasonic_interpreter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("waferbot_gazebo"), '/launch/helpers', '/ultrasonic_interpreter_launch.py'
        ])
    )

    image_compressor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("waferbot_gazebo"), '/launch/helpers', '/image_compressor_launch.py'
        ])
    )

    return LaunchDescription([
        ultrasonic_interpreter,
        image_compressor,
    ])
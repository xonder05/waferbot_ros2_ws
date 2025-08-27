from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("gazebo"), '/launch/bridges', '/simulator_camera_launch.py'
        ])
    )

    servo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("gazebo"), '/launch/bridges', '/simulator_servo_launch.py'
        ])
    )

    ultrasonic = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("gazebo"), '/launch/bridges', '/simulator_ultrasonic_launch.py'
        ])
    )

    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("gazebo"), '/launch/bridges', '/simulator_imu_launch.py'
        ])
    )

    line_following = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("gazebo"), '/launch/bridges', '/simulator_line_tracking_launch.py'
        ])
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("gazebo"), '/launch/bridges', '/simulator_lidar_launch.py'
        ])
    )

    return LaunchDescription([
        servo,
        camera,
        line_following,
        ultrasonic,
        imu,
        lidar,
    ])
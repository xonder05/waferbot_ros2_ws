import os
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml

def generate_launch_description():
    
    config_file = os.path.join(
        get_package_share_directory('waferbot_gazebo'),
        'config',
        'bridges_config.yaml'
    )

    with open(config_file, 'r') as f:
        config_contents = yaml.safe_load(f)

    # ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock
    clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            "/clock" + "@" +
            "rosgraph_msgs/msg/Clock" + "[" +
            "gz.msgs.Clock"
        ]
    )

    #ros2 run ros_gz_bridge parameter_bridge /simulator_imu@sensor_msgs/msg/Imu@gz.msgs.IMU
    imu = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            config_contents["imu_bridge"]["ros__parameters"]["topic"] + '@' + 
            config_contents["imu_bridge"]["ros__parameters"]["ros_message_type"] + '[' + 
            config_contents["imu_bridge"]["ros__parameters"]["gazebo_message_type"]
        ]
    )

    #ros2 run ros_gz_bridge parameter_bridge /simulator_lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan
    lidar = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            config_contents["lidar_bridge"]["ros__parameters"]["topic"] + '@' + 
            config_contents["lidar_bridge"]["ros__parameters"]["ros_message_type"] + '[' + 
            config_contents["lidar_bridge"]["ros__parameters"]["gazebo_message_type"]
        ]
    )

    #ros2 run ros_gz_bridge parameter_bridge /simulator_lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan
    ultrasonic = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            config_contents["ultrasonic_bridge"]["ros__parameters"]["topic"] + '@' + 
            config_contents["ultrasonic_bridge"]["ros__parameters"]["ros_message_type"] + '[' + 
            config_contents["ultrasonic_bridge"]["ros__parameters"]["gazebo_message_type"]
        ]
    )

    #ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@gz.msgs.Image /camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo
    camera = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            config_contents["camera_bridge"]["ros__parameters"]["topic"] + '@' + 
            config_contents["camera_bridge"]["ros__parameters"]["ros_message_type"] + '[' + 
            config_contents["camera_bridge"]["ros__parameters"]["gazebo_message_type"]
        ]
    )

    return launch.LaunchDescription([
        clock,
        imu,
        lidar,
        ultrasonic,
        camera
    ])
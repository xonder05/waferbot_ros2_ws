# WaferBot

## Dependencies
- [ROS2 Jazzy](https://docs.ros.org/en/jazzy/index.html)
- [ros2_control](https://control.ros.org/jazzy/index.html)
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Nav2](https://docs.nav2.org/index.html)

## Instalation
Make sure to clone using `--recurse-submodules` since 
this repository depends on other ROS2 packages, that you can find in the `src/lib` folder.

If you have all the dependencies than you should be able to just clone the workspace, build with 
`colcon build` (don't forget to source) and run with 
`ros2 launch waferbot_bringup waferbot_launch.py`.

## Documentation

### waferbot_behaviors
Implements two nodes for map less movement.

**Motion executor** provides two action servers for drive 
and spin behavior. Listens to transformation topic and 
handles loop closure.

`ros2 launch waferbot_behaviors motion_executor_launch.py`

**Wandering node** implements fuzzy based wandering. 
Depends on motion executor for movement. Sends commands 
to servo and listens to ultrasonic measurements.

`ros2 launch waferbot_behaviors wandering_launch.py`

### waferbot_bringup
This is the main package to bringup the entire robot 
at once. 

**waferbot_lauhch.py** is a top level launch file
for starting the entire robot. Its most important 
argument is `targets`.
Using it you can select which parts of the stack will 
be started. Options are `real`, `sim`, `localization`, `wandering`, `static_map`, `live_mapping` and `navigation`. 
The argument can take any number of these in any order separated by spaces (except for real and simulation, wandering and navigation, those are exclusive, also note that navigation / wandering will not work without localization).

`ros2 launch waferbot_bringup waferobot_launch.py targets:="real localization live_mapping navigation"`

**real_robot_launch.py** connects low level launch files
from sensors and control packages together. It is being used by the top level waferbot_launch.py file. You can of course use this directly, just take into account you 
need to have robot_state_publisher running, otherwise this launch will fail.

`ros2 launch waferbot_bringup real_robot_launch.py`

**simulation_launch.py** is the same case as the one above. It is combination of launch files from gazebo and control packages. Just as before, requires robot_state_publisher to work properly.

`ros2 launch waferbot_bringup simulation_launch.py`

**rviz_launch.py** is pretty self explanatory. This file starts up rviz with one of two prepared configuration files.

`ros2 launch waferbot_bringup rviz_launch.py config_file:=[single_robot|multi_robot]`

### waferbot_control_bringup
This package handles starting ros2_control.
Hardware interfaces used as part of this robot are located in the `lib` folder. This package depends on `waferbot_description` package, because parts of configurations come from robot description. Launch files provided in this package are split into four, for the sake of modularity. This way exact same controller spawners can be used for both real and simulated robot.

`ros2 launch waferbot_control_bringup controller_manager_launch.py` \
`ros2 launch waferbot_control_bringup diff_drive_controller_launch.py` \
`ros2 launch waferbot_control_bringup forward_command_controller_launch.py` \
`ros2 launch waferbot_control_bringup joint_state_broadcaster_launch.py`

### waferbot_description
This package contains Xacro description of the robot. Multiple other packages depend on this one.
The description is being published using robot_state_publisher node. Xacro translation and node startup handles **robot_state_publisher_launch.py**.

`ros2 launch waferbot_description robot_state_publisher_launch.py`

### waferbot_gazebo
This package contains everything needed to run waferbot model in simulator. 

**gazebo_launch.py** selects world and starts up simulator. 

`ros2 launch waferbot_gazebo gazebo_launch.py`

**spawn_robot_launch.py** spawns robot, whose description it gets from topic, into running simulator.

`ros2 launch waferbot_gazebo spanw_robot_launch.py`

**bridges_launch.py** using `ros_gz_bridge` package start bridge that translates messages between gazebo transport and ros2.

`ros2 launch waferbot_gazebo bridges_launch.py`

Last but not least, this package contains implementation of two helper nodes. They can be started individually or together.  

`ros2 launch waferbot_gazebo image_compressor_launch.py` \
`ros2 launch waferbot_gazebo ultrasonic_interpreted_launch.py` \
`ros2 launch waferbot_gazebo helpers_launch.py`

### waferbot_localization_bringup
This package depends on the `robot_localization` package.
**ekf_fusion_launch.py** fuses odometry data from motor encoders and imu using extended kalman filter.

`ros2 launch waferbot_localization_bringup ekf_fusion_launch.py`

### waferbot_mapping_bringup
This package provides two launch files related to mapping.
**live_mapping_launch.py** for slam mapping and **static_map_launch.py** that loads map from file.

`ros2 launch waferbot_mapping_bringup live_mapping_launch.py` \
`ros2 launch waferbot_mapping_bringup static_map_launch.py`

### waferbot_msgs
This package contains custom interfaces. The `motion_executor_node.py` needs two action messages for spin and drive.

### waferbot_navigation_bringup
This package contains launch and configuration for Nav2 navigation stack.

`ros2 launch waferbot_navigation_bringup navigation_launch.py`

### waferbot_sensors
This package implements nodes for controlling sensors. There are three of them for camera, imu and ultrasonic. Each of these has its own launch file.

`ros2 launch waferbot_sensors camera_launch.py` \
`ros2 launch waferbot_sensors imu_launch.py` \
`ros2 launch waferbot_sensors ultrasonic_launch.py`

Apart from these, this package also contains **ydlidar_launch.py** for starting ydlidar. Therefore this package depends on ydlidar driver located in the `lib` folder.

`ros2 launch waferbot_sensors ydlidar_launch.py`

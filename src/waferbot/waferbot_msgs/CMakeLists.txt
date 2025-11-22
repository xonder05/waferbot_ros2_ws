cmake_minimum_required(VERSION 3.8)
project(waferbot_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
)

set(srv_files
)

set(action_files
  "action/Drive.action"
  "action/Spin.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

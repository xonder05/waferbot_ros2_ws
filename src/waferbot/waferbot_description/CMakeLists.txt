cmake_minimum_required(VERSION 3.8)
project(waferbot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch meshes xacro
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

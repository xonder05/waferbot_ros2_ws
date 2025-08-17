cmake_minimum_required(VERSION 3.8)
project(robot_description)

# find dependencies
find_package(ament_cmake REQUIRED)

# files to copy during install
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY description/
  DESTINATION share/${PROJECT_NAME}/description
)

ament_package()

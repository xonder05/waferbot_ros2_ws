cmake_minimum_required(VERSION 3.8)
project(waferbot_description)

# find dependencies
find_package(ament_cmake REQUIRED)

# files to copy during install
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY description/
  DESTINATION share/${PROJECT_NAME}/description
)

ament_package()

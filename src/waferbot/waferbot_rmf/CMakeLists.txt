cmake_minimum_required(VERSION 3.8)

project(waferbot_rmf)

find_package(ament_cmake REQUIRED)

ament_python_install_package(${PROJECT_NAME})

file(GLOB_RECURSE traffic_editor_paths "maps/*.building.yaml")

foreach(path ${traffic_editor_paths})

  # Get the output world name
  string(REGEX REPLACE "\\.[^.]*\.[^.]*$" "" no_extension_path ${path})
  string(REGEX MATCH "[^\/]+$" world_name  ${no_extension_path})

  set(map_path ${path})
  set(output_world_name ${world_name})
  set(output_dir ${CMAKE_CURRENT_BINARY_DIR}/maps/${output_world_name})

  ##############################################################################
  # Generate the nav graphs
  ##############################################################################

  set(output_nav_graphs_dir ${output_dir}/nav_graphs/)
  set(output_nav_graphs_phony ${output_nav_graphs_dir}/phony)
  add_custom_command(
    OUTPUT ${output_nav_graphs_phony}
    COMMAND ros2 run rmf_building_map_tools building_map_generator nav ${map_path} ${output_nav_graphs_dir}
    DEPENDS ${map_path}
  )

  add_custom_target(generate_${output_world_name}_nav_graphs ALL
    DEPENDS ${output_nav_graphs_phony}
  )

  install(
    DIRECTORY ${output_dir}
    DESTINATION share/${PROJECT_NAME}/maps
  )

endforeach()

install(PROGRAMS
  ${PROJECT_NAME}/fleet_adapter.py
  ${PROJECT_NAME}/robot_adapter.py
  ${PROJECT_NAME}/mock_dispenser_ingestor.py
  ${PROJECT_NAME}/rmf_visualization_fixer.py
  ${PROJECT_NAME}/task_requester.py
  ${PROJECT_NAME}/task_state_logger.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

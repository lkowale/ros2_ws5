cmake_minimum_required(VERSION 3.8)
project(field_bt_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(field_nav REQUIRED)
find_package(rcl_interfaces REQUIRED)

set(dependencies
  rclcpp
  behaviortree_cpp
  geographic_msgs
  geometry_msgs
  field_nav
  rcl_interfaces
)

# LoadFieldProgress BT node
add_library(load_field_progress_bt_node SHARED
  src/load_field_progress.cpp
)
ament_target_dependencies(load_field_progress_bt_node ${dependencies})
target_link_libraries(load_field_progress_bt_node nlohmann_json::nlohmann_json)
target_compile_definitions(load_field_progress_bt_node PRIVATE BT_PLUGIN_EXPORT)

# SaveFieldProgress BT node
add_library(save_field_progress_bt_node SHARED
  src/save_field_progress.cpp
)
ament_target_dependencies(save_field_progress_bt_node ${dependencies})
target_link_libraries(save_field_progress_bt_node nlohmann_json::nlohmann_json)
target_compile_definitions(save_field_progress_bt_node PRIVATE BT_PLUGIN_EXPORT)

# NextFieldLine BT node
add_library(next_field_line_bt_node SHARED
  src/next_field_line.cpp
)
ament_target_dependencies(next_field_line_bt_node ${dependencies})
target_link_libraries(next_field_line_bt_node nlohmann_json::nlohmann_json)
target_compile_definitions(next_field_line_bt_node PRIVATE BT_PLUGIN_EXPORT)

# GetTurnSequenceFile BT node
add_library(get_turn_sequence_bt_node SHARED
  src/get_turn_sequence.cpp
)
ament_target_dependencies(get_turn_sequence_bt_node ${dependencies})
target_compile_definitions(get_turn_sequence_bt_node PRIVATE BT_PLUGIN_EXPORT)

# SetSwathLateralOffset BT node
add_library(set_swath_lateral_offset_bt_node SHARED
  src/set_swath_lateral_offset.cpp
)
ament_target_dependencies(set_swath_lateral_offset_bt_node ${dependencies})
target_compile_definitions(set_swath_lateral_offset_bt_node PRIVATE BT_PLUGIN_EXPORT)

# Install libraries
install(TARGETS
  load_field_progress_bt_node
  save_field_progress_bt_node
  next_field_line_bt_node
  get_turn_sequence_bt_node
  set_swath_lateral_offset_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(
  load_field_progress_bt_node
  save_field_progress_bt_node
  next_field_line_bt_node
  get_turn_sequence_bt_node
  set_swath_lateral_offset_bt_node
)
ament_export_dependencies(${dependencies})

ament_package()

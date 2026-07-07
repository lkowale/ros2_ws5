cmake_minimum_required(VERSION 3.8)
project(one_line_progress_bt_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

set(dependencies
  rclcpp
  behaviortree_cpp
  geographic_msgs
  geometry_msgs
)

# SaveOneLineProgress BT node
add_library(save_one_line_progress_bt_node SHARED
  src/save_one_line_progress.cpp
)
ament_target_dependencies(save_one_line_progress_bt_node ${dependencies})
target_link_libraries(save_one_line_progress_bt_node nlohmann_json::nlohmann_json)
target_compile_definitions(save_one_line_progress_bt_node PRIVATE BT_PLUGIN_EXPORT)

# LoadOneLineProgress BT node
add_library(load_one_line_progress_bt_node SHARED
  src/load_one_line_progress.cpp
)
ament_target_dependencies(load_one_line_progress_bt_node ${dependencies})
target_link_libraries(load_one_line_progress_bt_node nlohmann_json::nlohmann_json)
target_compile_definitions(load_one_line_progress_bt_node PRIVATE BT_PLUGIN_EXPORT)

# ClearOneLineProgress BT node
add_library(clear_one_line_progress_bt_node SHARED
  src/clear_one_line_progress.cpp
)
ament_target_dependencies(clear_one_line_progress_bt_node ${dependencies})
target_compile_definitions(clear_one_line_progress_bt_node PRIVATE BT_PLUGIN_EXPORT)

# SetStage BT node
add_library(set_stage_bt_node SHARED
  src/set_stage.cpp
)
ament_target_dependencies(set_stage_bt_node ${dependencies})
target_compile_definitions(set_stage_bt_node PRIVATE BT_PLUGIN_EXPORT)

# CheckStage BT nodes (CheckStageIs, CheckStagePast)
add_library(check_stage_bt_node SHARED
  src/check_stage.cpp
)
ament_target_dependencies(check_stage_bt_node ${dependencies})
target_compile_definitions(check_stage_bt_node PRIVATE BT_PLUGIN_EXPORT)

# Install libraries
install(TARGETS
  save_one_line_progress_bt_node
  load_one_line_progress_bt_node
  clear_one_line_progress_bt_node
  set_stage_bt_node
  check_stage_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(
  save_one_line_progress_bt_node
  load_one_line_progress_bt_node
  clear_one_line_progress_bt_node
  set_stage_bt_node
  check_stage_bt_node
)
ament_export_dependencies(${dependencies})

ament_package()

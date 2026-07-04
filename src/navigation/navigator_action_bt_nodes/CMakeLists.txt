cmake_minimum_required(VERSION 3.8)
project(navigator_action_bt_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(solbot5_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)

set(dependencies
  rclcpp
  rclcpp_action
  behaviortree_cpp
  nav2_behavior_tree
  solbot5_msgs
  geographic_msgs
  geometry_msgs
  tf2_ros
  tf2
)

# ApproachSwathAction BT node
add_library(approach_swath_action_bt_node SHARED
  src/approach_swath_action.cpp
)
target_compile_definitions(approach_swath_action_bt_node PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(approach_swath_action_bt_node ${dependencies})

# RunSwathAction BT node
add_library(run_swath_action_bt_node SHARED
  src/run_swath_action.cpp
)
target_compile_definitions(run_swath_action_bt_node PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(run_swath_action_bt_node ${dependencies})

# ReverseGeoPoints BT node
add_library(reverse_geo_points_bt_node SHARED
  src/reverse_geo_points.cpp
)
target_compile_definitions(reverse_geo_points_bt_node PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(reverse_geo_points_bt_node ${dependencies})

# GetGeoPointFromVector BT node
add_library(get_geo_point_from_vector_bt_node SHARED
  src/get_geo_point_from_vector.cpp
)
target_compile_definitions(get_geo_point_from_vector_bt_node PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(get_geo_point_from_vector_bt_node ${dependencies})

# ComputeArcTurnPoses BT node
add_library(compute_arc_turn_poses_bt_node SHARED
  src/compute_arc_turn_poses.cpp
)
target_compile_definitions(compute_arc_turn_poses_bt_node PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(compute_arc_turn_poses_bt_node ${dependencies})

install(TARGETS
  approach_swath_action_bt_node
  run_swath_action_bt_node
  reverse_geo_points_bt_node
  get_geo_point_from_vector_bt_node
  compute_arc_turn_poses_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(
  approach_swath_action_bt_node
  run_swath_action_bt_node
  reverse_geo_points_bt_node
  get_geo_point_from_vector_bt_node
  compute_arc_turn_poses_bt_node
)
ament_export_dependencies(${dependencies})

ament_package()

cmake_minimum_required(VERSION 3.8)
project(one_line_nav)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(pluginlib REQUIRED)
find_package(solbot5_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(include)

set(nav_dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  nav2_core
  nav2_util
  nav2_behavior_tree
  pluginlib
  solbot5_msgs
  behaviortree_cpp
  geometry_msgs
  geographic_msgs
)

# Navigator plugin
add_library(one_line_navigator SHARED
  src/one_line_navigator.cpp
)
ament_target_dependencies(one_line_navigator ${nav_dependencies})
target_link_libraries(one_line_navigator nlohmann_json::nlohmann_json)

# BT node plugins
add_library(one_line_bt_nodes SHARED
  src/bt_nodes.cpp
)
target_compile_definitions(one_line_bt_nodes PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(one_line_bt_nodes
  behaviortree_cpp
  geometry_msgs
  geographic_msgs
  robot_localization
  rclcpp
)

pluginlib_export_plugin_description_file(nav2_core navigator_plugins.xml)

install(TARGETS one_line_navigator one_line_bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

install(FILES navigator_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(one_line_navigator one_line_bt_nodes)
ament_export_dependencies(${nav_dependencies})
ament_package()

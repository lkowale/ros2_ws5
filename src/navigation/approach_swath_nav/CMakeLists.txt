cmake_minimum_required(VERSION 3.8)
project(approach_swath_nav)

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
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  nav2_core
  nav2_util
  nav2_behavior_tree
  pluginlib
  solbot5_msgs
  behaviortree_cpp
  geographic_msgs
  geometry_msgs
)

# Navigator plugin library
add_library(approach_swath_navigator SHARED
  src/approach_swath_navigator.cpp
)
ament_target_dependencies(approach_swath_navigator ${dependencies})

# Install libraries
install(TARGETS
  approach_swath_navigator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install behavior trees
install(DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

# Install plugin description files
pluginlib_export_plugin_description_file(nav2_core navigator_plugins.xml)

ament_export_include_directories(include)
ament_export_libraries(approach_swath_navigator)
ament_export_dependencies(${dependencies})

ament_package()

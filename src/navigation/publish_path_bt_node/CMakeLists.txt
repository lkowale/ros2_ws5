cmake_minimum_required(VERSION 3.8)
project(publish_path_bt_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(dependencies
  rclcpp
  behaviortree_cpp
  nav_msgs
  geometry_msgs
)

add_library(publish_path_bt_node SHARED
  src/publish_path.cpp
)
ament_target_dependencies(publish_path_bt_node ${dependencies})
target_compile_definitions(publish_path_bt_node PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS publish_path_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(publish_path_bt_node)
ament_export_dependencies(${dependencies})
ament_package()

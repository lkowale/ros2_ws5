cmake_minimum_required(VERSION 3.8)
project(drive_arc)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(solbot5_msgs REQUIRED)

set(deps
  rclcpp rclcpp_lifecycle
  nav2_core nav2_behaviors nav2_behavior_tree nav2_util nav2_msgs
  pluginlib geometry_msgs nav_msgs tf2 solbot5_msgs
)

include_directories(include)

# Behavior server plugin
add_library(drive_arc_behavior SHARED
  src/drive_arc_behavior.cpp
)
ament_target_dependencies(drive_arc_behavior ${deps})
pluginlib_export_plugin_description_file(nav2_core behavior_plugin.xml)

# BT action node plugin
add_library(drive_arc_bt_node SHARED
  src/drive_arc_bt_node.cpp
)
target_compile_definitions(drive_arc_bt_node PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(drive_arc_bt_node
  behaviortree_cpp nav2_behavior_tree solbot5_msgs rclcpp)

install(TARGETS drive_arc_behavior drive_arc_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES behavior_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(drive_arc_behavior drive_arc_bt_node)
ament_export_dependencies(${deps})
ament_package()

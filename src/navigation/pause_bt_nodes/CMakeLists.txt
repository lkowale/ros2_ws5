cmake_minimum_required(VERSION 3.8)
project(pause_bt_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(solbot5_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)

include_directories(include)

set(dependencies
  rclcpp
  solbot5_msgs
  behaviortree_cpp
)

# BT condition node plugin library for pause functionality
add_library(pause_condition_bt_node SHARED
  src/pause_condition.cpp
)
ament_target_dependencies(pause_condition_bt_node ${dependencies})
target_compile_definitions(pause_condition_bt_node PRIVATE BT_PLUGIN_EXPORT)

# Install libraries
install(TARGETS
  pause_condition_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(pause_condition_bt_node)
ament_export_dependencies(${dependencies})

ament_package()

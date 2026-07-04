cmake_minimum_required(VERSION 3.8)
project(universal_rs_planner)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ompl REQUIRED)

if(NOT TARGET ompl::ompl)
  add_library(ompl::ompl SHARED IMPORTED)
  set_target_properties(ompl::ompl PROPERTIES
    IMPORTED_LOCATION "/opt/ros/jazzy/lib/x86_64-linux-gnu/libompl.so"
    INTERFACE_INCLUDE_DIRECTORIES "${OMPL_INCLUDE_DIRS}")
endif()

set(dependencies
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_util
  nav2_costmap_2d
  nav_msgs
  geometry_msgs
  tf2_ros
  pluginlib
)

add_library(universal_rs_planner_plugin SHARED
  src/universal_rs_planner.cpp
)

target_include_directories(universal_rs_planner_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(universal_rs_planner_plugin ${dependencies})
target_link_libraries(universal_rs_planner_plugin ompl::ompl)
target_include_directories(universal_rs_planner_plugin PRIVATE ${OMPL_INCLUDE_DIRS})
target_compile_definitions(universal_rs_planner_plugin PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(nav2_core global_planner_plugin.xml)

install(TARGETS universal_rs_planner_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES global_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(universal_rs_planner_plugin)
ament_export_dependencies(${dependencies})
ament_package()

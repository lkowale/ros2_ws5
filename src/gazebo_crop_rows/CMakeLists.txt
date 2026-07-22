cmake_minimum_required(VERSION 3.8)
project(gazebo_crop_rows)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robot_localization REQUIRED)

set(CMAKE_PREFIX_PATH
  "/opt/ros/jazzy/opt/gz_transport_vendor"
  "/opt/ros/jazzy/opt/gz_msgs_vendor"
  ${CMAKE_PREFIX_PATH})

find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_executable(crop_row_spawner src/crop_row_spawner.cpp)

target_include_directories(crop_row_spawner PRIVATE
  /opt/ros/jazzy/opt/gz_transport_vendor/include/gz/transport13
  /opt/ros/jazzy/opt/gz_msgs_vendor/include/gz/msgs10
  /usr/include/nlohmann
)

ament_target_dependencies(crop_row_spawner
  rclcpp
  tf2_ros
  geometry_msgs
  nav_msgs
  robot_localization
)

target_link_libraries(crop_row_spawner
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

install(TARGETS crop_row_spawner
  DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  gazebo_crop_rows/gz_rl_recorder.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gz_rl_recorder
)

ament_package()

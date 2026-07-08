cmake_minimum_required(VERSION 3.8)
project(tool_slider)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

set(deps rclcpp std_msgs geometry_msgs tf2_ros tf2)

# plant_row_detector_sim — fake plant row offset publisher (sine wave / static)
add_executable(plant_row_detector_sim src/plant_row_detector_sim.cpp)
ament_target_dependencies(plant_row_detector_sim ${deps})

# tool_slider_controller — reads row offset, commands toolbar position
add_executable(tool_slider_controller src/tool_slider_controller.cpp)
ament_target_dependencies(tool_slider_controller ${deps})

# tool_actuator_sim — first-order lag actuator + TF tool_bar broadcaster
add_executable(tool_actuator_sim src/tool_actuator_sim.cpp)
ament_target_dependencies(tool_actuator_sim ${deps})

install(TARGETS
  plant_row_detector_sim
  tool_slider_controller
  tool_actuator_sim
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

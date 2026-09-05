cmake_minimum_required(VERSION 3.8)
project(crop_row_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

set(deps rclcpp std_msgs sensor_msgs cv_bridge image_transport)

add_executable(crop_row_vision_node src/crop_row_vision_node.cpp)
ament_target_dependencies(crop_row_vision_node ${deps})
target_link_libraries(crop_row_vision_node ${OpenCV_LIBS})

install(TARGETS
  crop_row_vision_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

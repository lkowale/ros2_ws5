cmake_minimum_required(VERSION 3.8)
project(solbot5_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/RunOneLine.action"
  "action/DriveArc.action"
  "action/RunRsTest.action"
  DEPENDENCIES std_msgs geometry_msgs geographic_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

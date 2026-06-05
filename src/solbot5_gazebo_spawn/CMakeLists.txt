cmake_minimum_required(VERSION 3.8)
project(solbot5_gazebo_spawn)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY configs launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/ackermann_cmd_vel_preprocessor.py
    scripts/odom_covariance_injector.py
    scripts/imu_covariance_injector.py
    scripts/sim_relposned_publisher.py
    scripts/sim_gps_fix_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

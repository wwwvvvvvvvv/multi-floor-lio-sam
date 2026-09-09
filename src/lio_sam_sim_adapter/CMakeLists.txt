cmake_minimum_required(VERSION 3.8)
project(lio_sam_sim_adapter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(
  lidar_time_converter
  src/lidar_time_converter.cpp
)

ament_target_dependencies(
  lidar_time_converter
  rclcpp
  sensor_msgs
)

install(
  TARGETS lidar_time_converter
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

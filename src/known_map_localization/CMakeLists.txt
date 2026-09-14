cmake_minimum_required(VERSION 3.8)
project(known_map_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================
# ROS2 dependencies
# ============================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(pcl_conversions REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)

# Ignition / Gazebo ground truth
find_package(ignition-transport11 REQUIRED)

# ============================================================
# PCL
# ============================================================
find_package(PCL REQUIRED COMPONENTS
  common
  io
  filters
  registration
)

include_directories(
  ${PCL_INCLUDE_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

# ============================================================
# 1. Known Map Publisher
#
# Load GlobalMap.pcd and publish /known_map
# ============================================================
add_executable(
  known_map_publisher
  src/known_map_publisher.cpp
)

ament_target_dependencies(
  known_map_publisher
  rclcpp
  sensor_msgs
  pcl_conversions
)

target_link_libraries(
  known_map_publisher
  ${PCL_LIBRARIES}
)

# ============================================================
# 2. NDT Localizer
#
# Known-map NDT localization
# ============================================================
add_executable(
  ndt_localizer
  src/ndt_localizer.cpp
)

ament_target_dependencies(
  ndt_localizer
  rclcpp
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
  nav_msgs
  std_msgs
  pcl_conversions
  tf2_ros
)

target_link_libraries(
  ndt_localizer
  ${PCL_LIBRARIES}
)

# ============================================================
# 3. Localization Evaluator
#
# Compare localization result with Ground Truth
# ============================================================
add_executable(
  localization_evaluator
  src/localization_evaluator.cpp
)

ament_target_dependencies(
  localization_evaluator
  rclcpp
  geometry_msgs
  tf2_msgs
)

# ============================================================
# 4. Gazebo World Pose Ground Truth
#
# Read Gazebo world pose and publish independent Ground Truth
# ============================================================
add_executable(
  world_pose_ground_truth
  src/world_pose_ground_truth.cpp
)

ament_target_dependencies(
  world_pose_ground_truth
  rclcpp
  geometry_msgs
)

target_link_libraries(
  world_pose_ground_truth
  ignition-transport11::core
)

# ============================================================
# 5. Odom TF Broadcaster
#
# Convert /odom pose into:
#
# odom -> base_link
#
# This is the lower half of the future standard TF tree:
#
# map
#  |
# odom
#  |
# base_link
#  |
# lidar_link
# ============================================================
add_executable(
  odom_tf_broadcaster
  src/odom_tf_broadcaster.cpp
)

ament_target_dependencies(
  odom_tf_broadcaster
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
)

# ============================================================
# Install executables
# ============================================================
install(
  TARGETS
  known_map_publisher
  ndt_localizer
  localization_evaluator
  world_pose_ground_truth
  odom_tf_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================================
# Install launch and config
# ============================================================
install(
  DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# ============================================================
# Package
# ============================================================
ament_package()

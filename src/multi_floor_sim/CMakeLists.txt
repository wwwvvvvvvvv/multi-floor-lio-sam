cmake_minimum_required(VERSION 3.8)
project(multi_floor_sim)

find_package(ament_cmake REQUIRED)

# 安装 launch、world、model、URDF 和配置文件
install(
  DIRECTORY
    launch
    worlds
    models
    urdf
    config
  DESTINATION share/${PROJECT_NAME}
)

# 安装 Floor Map Manager Python 节点
install(
  PROGRAMS
    scripts/floor_map_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

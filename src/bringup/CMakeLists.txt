cmake_minimum_required(VERSION 3.8)
project(bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# 安装Python脚本到正确位置
install(PROGRAMS
  scripts/costmap_monitor.py
  scripts/restart_manager.py
  scripts/scan_monitor.py
  scripts/map_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件和配置文件
install(DIRECTORY
  launch
  config
  map
  behavior_trees
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
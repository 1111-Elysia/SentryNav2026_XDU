cmake_minimum_required(VERSION 3.8)
project(rosbag_record)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(PROGRAMS
  scripts/rosbag_record_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config map
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

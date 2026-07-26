cmake_minimum_required(VERSION 3.8)
project(serial_comm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sentry_msgs REQUIRED)   

include_directories(include)

add_executable(serial_comm_node src/serial_comm_node.cpp)
ament_target_dependencies(serial_comm_node
  rclcpp
  geometry_msgs
  std_msgs
  sentry_msgs
)

install(TARGETS
  serial_comm_node  
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

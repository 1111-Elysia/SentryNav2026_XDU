cmake_minimum_required(VERSION 3.8)
project(can_comm)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sentry_msgs REQUIRED)
find_package(rm2_referee_msgs REQUIRED)

add_executable(can_comm_node
  src/can_comm.cpp
)

ament_target_dependencies(can_comm_node
  rclcpp
  geometry_msgs
  sentry_msgs
  rm2_referee_msgs
)

add_subdirectory(librm)
target_link_libraries(can_comm_node rm)

install(TARGETS can_comm_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

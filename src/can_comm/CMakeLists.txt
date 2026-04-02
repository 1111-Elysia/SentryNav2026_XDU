cmake_minimum_required(VERSION 3.8)
project(can_comm)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sentry_msgs REQUIRED)
find_package(rm_referee_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(can_comm_node
  src/can_comm.cpp
)

add_executable(vyaw_tf_yaw_controller_node
  src/vyaw_tf_yaw_controller_node.cpp
)

target_include_directories(can_comm_node PRIVATE
  include
)

ament_target_dependencies(can_comm_node
  rclcpp
  geometry_msgs
  std_msgs
  sentry_msgs
  rm_referee_msgs
  tf2
  tf2_ros
)

ament_target_dependencies(vyaw_tf_yaw_controller_node
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
)

add_subdirectory(librm)
target_link_libraries(can_comm_node rm)

install(TARGETS can_comm_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS vyaw_tf_yaw_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

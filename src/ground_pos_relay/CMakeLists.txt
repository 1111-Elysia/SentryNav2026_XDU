cmake_minimum_required(VERSION 3.8)
project(ground_pos_relay)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rm_referee_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(ground_pos_relay_node src/ground_pos_relay_node.cpp)
target_include_directories(ground_pos_relay_node PRIVATE include)
ament_target_dependencies(ground_pos_relay_node
  rclcpp
  rm_referee_msgs
  std_msgs
)

install(TARGETS ground_pos_relay_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

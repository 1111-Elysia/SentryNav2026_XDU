cmake_minimum_required(VERSION 3.8)
project(sentry_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2 REQUIRED)

# Build TF and Odom publisher node
add_executable(tf_odom_publisher src/tf_odom_publisher.cpp)
ament_target_dependencies(tf_odom_publisher
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  tf2_msgs
  tf2
)

install(TARGETS
  tf_odom_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
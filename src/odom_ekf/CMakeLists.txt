cmake_minimum_required(VERSION 3.8)
project(odom_ekf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(odom_preprocessor src/odom_preprocessor.cpp)
ament_target_dependencies(odom_preprocessor rclcpp nav_msgs)

install(TARGETS odom_preprocessor DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()

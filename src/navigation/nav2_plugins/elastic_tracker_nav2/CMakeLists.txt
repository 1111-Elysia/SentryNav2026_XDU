cmake_minimum_required(VERSION 3.5)
project(elastic_tracker_nav2)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_smac_planner REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Elastic Planner library (header-only core + compiled plugin)
add_library(elastic_planner SHARED
  src/elastic_planner.cpp
  src/corridor_2d.cpp
  src/minco_optimizer.cpp
)

ament_target_dependencies(elastic_planner
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav2_util
  tf2_ros
  tf2_geometry_msgs
  nav2_smac_planner
  geometry_msgs
  nav_msgs
  pluginlib
)

target_link_libraries(elastic_planner Eigen3::Eigen)

# Allow plugin to find itself
target_include_directories(elastic_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# Install
install(TARGETS elastic_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(nav2_core plugins.xml)

ament_export_include_directories(include)
ament_export_libraries(elastic_planner)
ament_export_dependencies(
  nav2_core
  nav2_costmap_2d
  Eigen3
)
ament_package()

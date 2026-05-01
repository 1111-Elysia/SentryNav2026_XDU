cmake_minimum_required(VERSION 3.8)
project(d435_process_pcl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

add_executable(depth_to_pcl_node src/depth_to_pcl_node.cpp)
target_include_directories(depth_to_pcl_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(depth_to_pcl_node SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(depth_to_pcl_node ${PCL_LIBRARIES})

ament_target_dependencies(
  depth_to_pcl_node
  "rclcpp"
  "sensor_msgs"
  "cv_bridge"
  "image_geometry"
  "pcl_conversions"
  "pcl_ros"
  "message_filters"
)

install(TARGETS
  depth_to_pcl_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

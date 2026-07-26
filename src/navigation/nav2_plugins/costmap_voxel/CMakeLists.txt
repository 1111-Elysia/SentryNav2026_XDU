cmake_minimum_required(VERSION 3.10)
project(costmap_voxel)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

add_library(costmap_voxel_layer SHARED
  src/voxel_filter_layer.cpp
)

target_include_directories(costmap_voxel_layer PUBLIC
  include
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(costmap_voxel_layer
  rclcpp
  pluginlib
  sensor_msgs
  nav2_costmap_2d
  pcl_conversions
)

target_link_libraries(costmap_voxel_layer
  ${PCL_LIBRARIES}
)

pluginlib_export_plugin_description_file(nav2_costmap_2d resource/costmap_voxel_plugins.xml)

install(TARGETS costmap_voxel_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES
  resource/costmap_voxel_plugins.xml
  DESTINATION share/costmap_voxel
)

# ament resource index marker
install(FILES resource/costmap_voxel
  DESTINATION share/ament_index/resource_index/packages
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(multi_odom_ekf)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(fusion_node src/fusion_node.cpp)
ament_target_dependencies(fusion_node
    rclcpp
    nav_msgs
    geometry_msgs
    tf2
    tf2_geometry_msgs
)

install(TARGETS fusion_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()

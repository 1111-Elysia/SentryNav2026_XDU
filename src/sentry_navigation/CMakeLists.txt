cmake_minimum_required(VERSION 3.8)
project(sentry_navigation)

# ===== 编译选项 =====
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ===== 查找依赖包 =====
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_msgs REQUIRED)

# ===== 查找 yaml-cpp (新增) =====
find_package(yaml-cpp REQUIRED)

# ===== 包含目录 =====
include_directories(
  include
  ${YAML_CPP_INCLUDE_DIRS}
)

# ===== 构建 TF Odom Publisher 节点 =====
add_executable(tf_odom_publisher 
  src/tf_odom_publisher.cpp
)

ament_target_dependencies(tf_odom_publisher
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  tf2_msgs
  tf2
)

# 链接 yaml-cpp 库
target_link_libraries(tf_odom_publisher
  yaml-cpp
)

# ===== 安装可执行文件 =====
install(TARGETS
  tf_odom_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# ===== 安装 launch 文件 =====
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# ===== 安装配置文件 =====
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# ===== 安装 RViz 配置 =====
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

# ===== 测试 =====
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ===== ament 打包 =====
ament_package()
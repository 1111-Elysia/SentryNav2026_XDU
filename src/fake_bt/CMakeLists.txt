cmake_minimum_required(VERSION 3.5)
project(fake_bt)

# 编译标准
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_CXX_STANDARD 17)

# 依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sentry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

# 可执行文件
add_executable(pub_point src/pub_point.cpp)
add_executable(pub_vw src/pub_vw.cpp)

# 依赖链接
ament_target_dependencies(pub_point
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  nav2_msgs
)

ament_target_dependencies(pub_vw
  rclcpp
  sentry_msgs
)

# 安装节点
install(TARGETS
  pub_point
  pub_vw
  DESTINATION lib/${PROJECT_NAME}
)

# 安装配置文件
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# 安装 launch 目录，以便 ros2 launch 能找到 launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装管理脚本到 share/<pkg>/scripts，这样 launch 中的 FindPackageShare 可以找到它
install(PROGRAMS
  scripts/fake_bt_manager.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_package()

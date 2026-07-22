cmake_minimum_required(VERSION 3.10)
project(sentry_nav_bt_test)

SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE "Release")
endif()

# 编译选项
add_compile_options(-Ofast -O3)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# TF2 相关 (BlackboardManager 需要)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sentry_msgs REQUIRED)

# 自定义消息包
find_package(rm_referee_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

# Include directories
include_directories(
  include
)

# ============================================
# 1. 创建库 (包含所有自定义节点实现)
# ============================================
add_library(sentry_nav_bt_test_nodes SHARED
  src/random_selector_node.cpp
  src/goal_selector_node.cpp
  src/patrol_nodes.cpp
  src/reliable_navigate_to_pose.cpp
  src/publish_vw_action.cpp
  src/referee_actions_common.cpp
  src/referee_engage_actions.cpp
  src/referee/control_topic_publishers.cpp
  src/blackboard/defaults.cpp
  src/navigation/waypoint_loader.cpp
  src/runtime_config.cpp
)

# 库的依赖
ament_target_dependencies(sentry_nav_bt_test_nodes
  rclcpp
  rclcpp_action
  rcl_interfaces
  behaviortree_cpp
  nav2_msgs
  nav2_util
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  sentry_msgs
  rm_referee_msgs
)
target_link_libraries(sentry_nav_bt_test_nodes
  nlohmann_json::nlohmann_json
)
# 允许 BT 插件导出
target_compile_definitions(sentry_nav_bt_test_nodes PRIVATE BT_PLUGIN_EXPORT)

# ============================================
# 2. 创建可执行文件 (Main)
# ============================================
add_executable(navigate_bt_node 
  src/sentry_nav_bt_test.cpp 
)

# 链接刚才创建的库
target_link_libraries(navigate_bt_node 
  sentry_nav_bt_test_nodes
  nlohmann_json::nlohmann_json
)

# Executable 的依赖 (主要是 BlackboardManager 在 main 中用到)
ament_target_dependencies(navigate_bt_node
  rclcpp
  rclcpp_action
  rcl_interfaces
  behaviortree_cpp
  nav2_msgs
  nav2_util
  geometry_msgs 
  std_msgs
  sentry_msgs
  tf2           
  tf2_ros
  tf2_geometry_msgs
  rm_referee_msgs
)

# ============================================
# 3. 安装规则
# ============================================
install(TARGETS
  sentry_nav_bt_test_nodes
  navigate_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

ament_export_include_directories(include)
ament_export_libraries(sentry_nav_bt_test_nodes)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  behaviortree_cpp
  nav2_msgs
  nav2_util
  tf2_ros
)

ament_package()

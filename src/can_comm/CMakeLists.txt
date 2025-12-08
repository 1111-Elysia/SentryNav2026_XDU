cmake_minimum_required(VERSION 3.8)
project(can_comm)

# ------------------------
# ROS2 依赖
# ------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sentry_msgs REQUIRED)

# ------------------------
# 指定 Python 路径，解决找不到 Python.h 问题
# ------------------------
set(PYTHON_EXECUTABLE /usr/bin/python3)
set(PYTHON_INCLUDE_DIR /usr/include/python3.10)
set(PYTHON_LIBRARY /usr/lib/x86_64-linux-gnu/libpython3.10.so)

# ------------------------
# pybind11 支持
# ------------------------
find_package(pybind11 REQUIRED)

# ------------------------
# 引入 librm（非 ROS 包）
# ------------------------
add_subdirectory(librm)

# ------------------------
# include 路径
# ------------------------
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../librm/libs
  ${CMAKE_CURRENT_SOURCE_DIR}/../librm/src
  ${PYTHON_INCLUDE_DIR}  # 确保 Python.h 可用
)

# ------------------------
# pybind11 模块：封装 librm 为 Python 可调用
# ------------------------
add_library(pyreferee MODULE src/pyreferee.cpp)
target_link_libraries(pyreferee PRIVATE rm pybind11::module ${PYTHON_LIBRARY})
set_target_properties(pyreferee PROPERTIES PREFIX "")  # 生成 pyreferee.so

# 安装 Python 模块
install(TARGETS pyreferee
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

# ------------------------
# 安装 scripts 下的 Python 节点
# ------------------------
install(PROGRAMS
  scripts/can_comm_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION lib/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml"
)

ament_package()

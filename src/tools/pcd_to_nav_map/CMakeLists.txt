cmake_minimum_required(VERSION 3.8)
project(pcd_to_nav2_map_converter)

# 设置C++标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

include_directories(
  ${PCL_INCLUDE_DIRS}
  include
)

# 查找PCL库
find_package(PCL 1.8 REQUIRED) # 根据你的PCL版本调整
find_package(nlohmann_json REQUIRED)

# 包含PCL头文件目录
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 添加可执行文件
add_executable(pcd_to_nav2_map src/pcd_to_nav_map.cpp) # 假设你的cpp文件在src目录下

# 链接PCL库
target_link_libraries(pcd_to_nav2_map ${PCL_LIBRARIES})

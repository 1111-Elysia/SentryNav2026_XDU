cmake_minimum_required(VERSION 3.16)

project(lightning)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

# set to release by default
set(CMAKE_BUILD_TYPE Release)

# -------------------------------------------------------------
# 导入第三方库，同时做一些全局编译选项配置
# 编译选项（优化过，不0再爆内存）
add_definitions("-DPCL_NO_PRECOMPILE")

# 基础 flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -fopenmp")

# Release 优化（无 -g，无 -ggdb）
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG -fopenmp -fPIC")

# Debug 模式才加 -g
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -ggdb -fopenmp -fPIC")

# C mode
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")


if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/libs)
# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/libs)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(cmake/packages.cmake)
find_package(rm_referee_msgs REQUIRED)

# msg 和 srv
set(MY_SRV_FILES "")
set(SRC_DIR ${PROJECT_SOURCE_DIR}/srv)
file(GLOB_RECURSE SRC_FILES ${SRC_DIR}/*.srv)
foreach (SRC_FILE ${SRC_FILES})
    file(RELATIVE_PATH REL_PATH ${SRC_DIR} ${SRC_FILE})
    list(APPEND MY_SRV_FILES ${PROJECT_SOURCE_DIR}:srv/${REL_PATH})
endforeach ()

rosidl_generate_interfaces(${PROJECT_NAME}
        ${MY_SRV_FILES}
        ${MY_MSG_FILES}
        ${MY_ACTION_FILES}
        DEPENDENCIES std_msgs geometry_msgs sensor_msgs nav_msgs
)

include_directories(
        ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)

add_subdirectory(thirdparty/livox_ros_driver)

add_subdirectory(src)

ament_package()

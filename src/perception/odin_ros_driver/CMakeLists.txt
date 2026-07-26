cmake_minimum_required(VERSION 3.5)

# Silence dev warnings coming from 3rd-party CMake configs (e.g. PCL) on newer CMake.
# Must be set before calling project()/enable_language().
if(POLICY CMP0144)
    set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)
    cmake_policy(SET CMP0144 NEW)
endif()
if(POLICY CMP0167)
    set(CMAKE_POLICY_DEFAULT_CMP0167 NEW)
    cmake_policy(SET CMP0167 NEW)
endif()

project(odin_ros_driver)

# ROS2-only build (ament_cmake). This package targets ROS2 Humble+ and is built with colcon.
add_compile_definitions(ROS2)
message(STATUS "Configuring ROS2-only (ament_cmake) build")

# Platform detection
execute_process(
    COMMAND uname -m
    OUTPUT_VARIABLE ARCH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(ARCH STREQUAL "x86_64")
    set(TARGET_PLATFORM "x86")
    message(STATUS "Detected x86_64 architecture")
elseif(ARCH MATCHES "arm|aarch64")
    set(TARGET_PLATFORM "arm")
    message(STATUS "Detected ARM architecture: ${ARCH}")
else()
    message(WARNING "Unsupported architecture: ${ARCH}. Using default settings")
    set(TARGET_PLATFORM "unknown")
endif()

# Set library path
set(LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib")
message(STATUS "Library directory: ${LIB_DIR}")

# Set library name based on platform
if(TARGET_PLATFORM STREQUAL "arm")
    set(LYD_HOST_API_LIB_NAME "lydHostApi_arm")
else()
    set(LYD_HOST_API_LIB_NAME "lydHostApi_amd")
endif()

# Find precompiled lydHostApi library
find_library(LYD_HOST_API_LIB
    NAMES 
        ${LYD_HOST_API_LIB_NAME}
        lib${LYD_HOST_API_LIB_NAME}.a
        lib${LYD_HOST_API_LIB_NAME}.so
    PATHS ${LIB_DIR}
    NO_DEFAULT_PATH
)

if(LYD_HOST_API_LIB)
    message(STATUS "Found lydHostApi library: ${LYD_HOST_API_LIB}")
else()
    file(GLOB LIB_FILES "${LIB_DIR}/lib${LYD_HOST_API_LIB_NAME}.*")
    if(LIB_FILES)
        message(STATUS "Found library files: ${LIB_FILES}")
        set(LYD_HOST_API_LIB ${LIB_FILES})
    else()
        message(FATAL_ERROR "Could not find precompiled lydHostApi library in ${LIB_DIR}")
    endif()
endif()

# Set common compile options
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set optimization flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

# Find common dependencies
find_package(PkgConfig REQUIRED)
# Prefer system OpenCV on Ubuntu/ROS2 to avoid mixing /usr/local OpenCV with cv_bridge.
# (Still allows overriding via -DOpenCV_DIR=...)
if(NOT DEFINED OpenCV_DIR)
    set(_SYSTEM_OPENCV_DIR "/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/opencv4")
    if(EXISTS "${_SYSTEM_OPENCV_DIR}/OpenCVConfig.cmake")
        set(OpenCV_DIR "${_SYSTEM_OPENCV_DIR}")
    endif()
    unset(_SYSTEM_OPENCV_DIR)
endif()
find_package(OpenCV REQUIRED CONFIG)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenSSL REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

# yaml-cpp exports an imported target in modern distros.
if(TARGET yaml-cpp::yaml-cpp)
    set(YAML_CPP_TARGET yaml-cpp::yaml-cpp)
else()
    set(YAML_CPP_TARGET yaml-cpp)
endif()

# Shared include directories
include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${LIBUSB_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Shared library list
set(COMMON_LIBS
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${OPENSSL_LIBRARIES}
    ${LIBUSB_LIBRARIES}
    pthread
    rt
    ${CMAKE_DL_LIBS}
    ${LYD_HOST_API_LIB}
)

# ===== ROS2 Configuration =====
message(STATUS "Configuring for ROS2 build")
    
    # Find all necessary ROS2 packages
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(visualization_msgs REQUIRED)
    find_package(cv_bridge REQUIRED)
    find_package(image_transport REQUIRED)
    find_package(pcl_conversions REQUIRED)
    find_package(message_filters REQUIRED)
    find_package(tf2 REQUIRED)
    find_package(tf2_ros REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(tf2_geometry_msgs REQUIRED)
    find_package(rm_referee_msgs REQUIRED)
    
    # Create executable
    add_executable(host_sdk_sample 
        src/host_sdk_sample.cpp
        src/yaml_parser.cpp  
        src/rawCloudRender.cpp
        src/camera_pose_visualization.cpp
        )
    
    # Link libraries
    target_link_libraries(host_sdk_sample
        ${COMMON_LIBS}
        ${YAML_CPP_TARGET}
        usb-1.0
    )
    
    # Add ROS2 dependencies
    ament_target_dependencies(host_sdk_sample
        rclcpp 
        std_msgs 
        sensor_msgs 
        nav_msgs 
        visualization_msgs
        cv_bridge
        image_transport
        tf2
        tf2_ros
        geometry_msgs
        tf2_geometry_msgs
        rm_referee_msgs
    )
    
    add_library(pointcloud_depth_converter_ros2 src/pointcloud_depth_converter.cpp)
    target_link_libraries(pointcloud_depth_converter_ros2
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
    )
    
    add_library(depth_image_ros2_node_lib src/depth_image_ros2_node.cpp)
    target_link_libraries(depth_image_ros2_node_lib
        pointcloud_depth_converter_ros2
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
    )
    
    ament_target_dependencies(depth_image_ros2_node_lib
        rclcpp
        sensor_msgs
        std_msgs
        cv_bridge
        image_transport
        pcl_conversions
        message_filters
    )

    add_executable(pcd2depth_ros2_node src/pcd2depth_ros2.cpp)
    target_link_libraries(pcd2depth_ros2_node
        depth_image_ros2_node_lib
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        ${YAML_CPP_TARGET}
    )
    
    ament_target_dependencies(pcd2depth_ros2_node
        rclcpp
        sensor_msgs
        std_msgs
        cv_bridge
        image_transport
        pcl_conversions
        message_filters
    )

    add_library(cloud_reprojector_ros2 src/cloud_reprojector.cpp)
    target_compile_definitions(cloud_reprojector_ros2 PRIVATE ROS2)
    target_link_libraries(cloud_reprojector_ros2
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
    )

    add_executable(cloud_reprojection_ros2_node src/cloud_reprojection_ros.cpp)
    target_compile_definitions(cloud_reprojection_ros2_node PRIVATE ROS2)
    target_link_libraries(cloud_reprojection_ros2_node
        cloud_reprojector_ros2
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        ${YAML_CPP_TARGET}
    )
    ament_target_dependencies(cloud_reprojection_ros2_node
        rclcpp
        sensor_msgs
        nav_msgs
        cv_bridge
        image_transport
        pcl_conversions
        message_filters
    )

    add_executable(image_overlay_node src/image_overlay_node.cpp)
    target_compile_definitions(image_overlay_node PRIVATE ROS2)
    target_link_libraries(image_overlay_node
        ${OpenCV_LIBS}
    )
    ament_target_dependencies(image_overlay_node
        rclcpp
        sensor_msgs
        cv_bridge
        image_transport
        message_filters
    )

    # Installation rules - ensure all install targets are defined before ament_package()
    # Install executable
    install(TARGETS 
        host_sdk_sample 
        pcd2depth_ros2_node
        cloud_reprojection_ros2_node
        image_overlay_node
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
    )
    
    # Install package.xml
    install(FILES package.xml
        DESTINATION share/${PROJECT_NAME}
    )
    
    # Install headers
    install(DIRECTORY include/
        DESTINATION include
    )
    # Install launch directory
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
        install(DIRECTORY launch/
            DESTINATION share/${PROJECT_NAME}/launch
        )
        message(STATUS "Installing launch directory to share/${PROJECT_NAME}/launch")
    else()
        message(WARNING "launch directory not found")
    endif()
    # Install config files
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
        install(DIRECTORY config/
            DESTINATION share/${PROJECT_NAME}/config
        )
    endif()
    
    # Install launch files
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
        install(DIRECTORY launch/
            DESTINATION share/${PROJECT_NAME}/launch
        )
    endif()
    ament_export_targets(export_${PROJECT_NAME})
    # Declare dependencies
    ament_export_dependencies(
        rclcpp 
        std_msgs 
        sensor_msgs 
        nav_msgs 
        visualization_msgs
        cv_bridge
        image_transport
        pcl_conversions
        message_filters
        tf2
        tf2_ros
        geometry_msgs
        tf2_geometry_msgs
    )
    
    ament_package()
    message(STATUS "Install targets added")


# ARM platform specific link options
if(TARGET_PLATFORM STREQUAL "arm")
    set_target_properties(host_sdk_sample PROPERTIES
        LINK_FLAGS "-Wl,--no-as-needed -Wl,--rpath=${LIB_DIR}"
    )
    message(STATUS "Adding ARM-specific link options and RPATH")
endif()

# Add debug information
message(STATUS "=======================================")
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "Target platform: ${TARGET_PLATFORM}")
message(STATUS "lydHostApi library: ${LYD_HOST_API_LIB}")
message(STATUS "libusb library: ${LIBUSB_LIBRARIES}")
message(STATUS "=======================================")


add_library(${PROJECT_NAME}.libs SHARED
        common/nav_state.cc
        common/constant.h
        common/options.cc
        common/params.cc

        io/yaml_io.cc
        io/file_io.cc

        core/lio/eskf.cc
        core/lio/laser_mapping.cc
        core/lio/pointcloud_preprocess.cc
        core/loop_closing/loop_closing.cc

        core/g2p5/g2p5_map.cc
        core/g2p5/g2p5.cc
        core/g2p5/g2p5_subgrid.cc

        core/maps/tiled_map.cc
        core/maps/tiled_map_chunk.cc

        core/localization/localization_result.cc
        core/localization/lidar_loc/lidar_loc.cc
        core/localization/lidar_loc/pclomp/ndt_omp.cpp
        core/localization/lidar_loc/pclomp/voxel_grid_covariance_omp.cpp

        core/localization/pose_graph/pgo.cc
        core/localization/pose_graph/pgo_impl.cc
        core/localization/pose_graph/pose_extrapolator.cc
        core/localization/localization.cpp

        utils/timer.cc
        utils/pointcloud_utils.cc

        ui/pangolin_window.cc
        ui/pangolin_window_impl.cc
        ui/ui_car.cc
        ui/ui_cloud.cc
        ui/ui_trajectory.cc

        core/system/slam.cc
        core/system/loc_system.cc

        wrapper/bag_io.cc
)

ament_target_dependencies(
        ${PROJECT_NAME}.libs
        "rclcpp"
        "std_msgs"
        "geometry_msgs"
        "sensor_msgs"
        "nav_msgs"
        "std_srvs"
        "message_filters"
        "pcl_conversions"
        "tf2_ros"
        "tf2"
)

add_dependencies(${PROJECT_NAME}.libs
        livox_interfaces2__rosidl_typesupport_cpp
        ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# 加入 rm2_referee_msgs 的生成头文件目录，供 app 源文件包含使用
include_directories(${PROJECT_SOURCE_DIR}/src/core/miao
        ${CMAKE_BINARY_DIR}/../rm2_referee_msgs/rosidl_generator_cpp
)

# 手动查找 rm2_referee_msgs 的库文件（绕过 find_package 失败问题）
find_library(RM2_REF_LIB 
    NAMES rm2_referee_msgs__rosidl_typesupport_cpp
    PATHS 
        ${CMAKE_BINARY_DIR}/../rm2_referee_msgs
        ${CMAKE_BINARY_DIR}/../rm2_referee_msgs/lib
    NO_DEFAULT_PATH
)
if (NOT RM2_REF_LIB)
    # 尝试在系统路径找
    find_library(RM2_REF_LIB NAMES rm2_referee_msgs__rosidl_typesupport_cpp)
endif()

if (RM2_REF_LIB)
    message(STATUS "Found rm2_referee_msgs lib: ${RM2_REF_LIB}")
else()
    message(WARNING "Could not find rm2_referee_msgs lib, linking might fail")
endif()

add_subdirectory(core/miao)

target_link_libraries(${PROJECT_NAME}.libs
        ${third_party_libs}
        ${PCL_LIBRARIES}
        miao.core
        miao.utils
        livox_interfaces2__rosidl_typesupport_cpp
        ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# 如果找到了 rm2_referee_msgs 库，也链接到主库
if (RM2_REF_LIB)
    target_link_libraries(${PROJECT_NAME}.libs ${RM2_REF_LIB})
endif()

install(TARGETS ${PROJECT_NAME}.libs miao.core miao.utils
        DESTINATION lib)

add_subdirectory(app)
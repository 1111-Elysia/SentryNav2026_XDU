//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <Sophus/se3.hpp> 

#include "core/system/loc_system.h"
#include "ui/pangolin_window.h"
#include "wrapper/ros_utils.h"

DEFINE_string(config, "./config/default.yaml", "配置文件");

/// 运行定位的测试
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;

    google::ParseCommandLineFlags(&argc, &argv, true);
    using namespace lightning;

    rclcpp::init(argc, argv);

    LocSystem::Options opt;
    LocSystem loc(opt);

    if (!loc.Init(FLAGS_config)) {
        LOG(ERROR) << "failed to init loc";
    }

    double roll_rad = 0.0 * M_PI / 180.0;
    double pitch_rad = 0.0 * M_PI / 180.0;
    double yaw_rad = 0.0 * M_PI / 180.0;
    
    // 创建旋转矩阵 (SO3)
    Sophus::SO3d rotation = Sophus::SO3d::rotZ(yaw_rad) * Sophus::SO3d::rotY(pitch_rad) * Sophus::SO3d::rotX(roll_rad);
    
    // 创建平移向量
    Eigen::Vector3d translation(0.0, 0.0, 0.0);
    
    // 创建 SE3 位姿
    Sophus::SE3d init_pose(rotation, translation);

    /// 默认起点开始定位
    LOG(INFO) << "初始位姿: " << init_pose.translation().transpose() 
          << ", 四元数: " << init_pose.unit_quaternion().coeffs().transpose();
    loc.SetInitPose(init_pose);
    loc.Spin();

    rclcpp::shutdown();

    return 0;
}
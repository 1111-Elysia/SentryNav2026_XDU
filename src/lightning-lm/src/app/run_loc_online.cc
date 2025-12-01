//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <Sophus/se3.hpp> 
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <future>
#include <thread>
#include <chrono>
#include <memory>
#include <vector>
#include <cmath>

#include "core/system/loc_system.h"
#include "ui/pangolin_window.h"
#include "wrapper/ros_utils.h"

// ...existing code...

DEFINE_string(config, "./config/default.yaml", "配置文件");

/// 运行定位的测试
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;

    google::ParseCommandLineFlags(&argc, &argv, true);
    using namespace lightning;

    rclcpp::init(argc, argv);

    // 通过话题 /init_pose 获取初始位姿（geometry_msgs::msg::PoseWithCovarianceStamped）
    auto param_node = rclcpp::Node::make_shared("init_pose_reader");
    auto prom = std::make_shared<std::promise<geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr>>();
    auto fut = prom->get_future();

    auto sub = param_node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/init_pose",
        rclcpp::QoS(1).transient_local(),
        [prom](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            try {
                prom->set_value(msg);
            } catch (...) {
                // already set or other error, 忽略
            }
        });

    // 等待消息到达，最多等待 1 秒（通过轮询 spin_some）
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr init_msg = nullptr;
    const auto timeout = std::chrono::milliseconds(1000);
    const auto start = std::chrono::steady_clock::now();
    while (fut.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
        rclcpp::spin_some(param_node);
        if (std::chrono::steady_clock::now() - start > timeout) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (fut.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        init_msg = fut.get();
    }

    LocSystem::Options opt;
    LocSystem loc(opt);

    if (!loc.Init(FLAGS_config)) {
        LOG(ERROR) << "failed to init loc";
    }

    double roll_rad = 0.0;
    double pitch_rad = 0.0;
    double yaw_rad = 0.0;
    Eigen::Vector3d translation(0.0, 0.0, 0.0);

    if (init_msg) {
        auto &p = init_msg->pose.pose.position;
        auto &o = init_msg->pose.pose.orientation;
        translation = Eigen::Vector3d(p.x, p.y, p.z);

        // 构造 Eigen 四元数（w, x, y, z）
        Eigen::Quaterniond q(o.w, o.x, o.y, o.z);
        Eigen::Matrix3d R = q.toRotationMatrix();

        // 从旋转矩阵获得 Sophus SO3
        Sophus::SO3d rotation(R);

        Sophus::SE3d init_pose(rotation, translation);

        LOG(INFO) << "从话题 /init_pose 读取初始位姿: " << init_pose.translation().transpose()
                  << ", 四元数: " << init_pose.unit_quaternion().coeffs().transpose();
        loc.SetInitPose(init_pose);
    } else {
        // 使用默认全 0 初始位姿
        roll_rad = 0.0 * M_PI / 180.0;
        pitch_rad = 0.0 * M_PI / 180.0;
        yaw_rad = 0.0 * M_PI / 180.0;

        Sophus::SO3d rotation = Sophus::SO3d::rotZ(yaw_rad) * Sophus::SO3d::rotY(pitch_rad) * Sophus::SO3d::rotX(roll_rad);
        Sophus::SE3d init_pose(rotation, translation);

        LOG(INFO) << "未收到 /init_pose，使用默认初始位姿: " << init_pose.translation().transpose()
                  << ", 四元数: " << init_pose.unit_quaternion().coeffs().transpose();
        loc.SetInitPose(init_pose);
    }

    loc.Spin();

    rclcpp::shutdown();

    return 0;
}

// ...existing code...
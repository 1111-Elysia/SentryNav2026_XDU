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
#include <rclcpp/rclcpp.hpp> 
#include "rm_referee_msgs/msg/robot_status.hpp"
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>


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

    // 订阅 /rm_referee/robot_status，获取 robot_id
    auto status_node = rclcpp::Node::make_shared("robot_status_reader");
    auto status_prom = std::make_shared<std::promise<rm_referee_msgs::msg::RobotStatus::SharedPtr>>();
    auto status_fut = status_prom->get_future();

    auto status_sub = status_node->create_subscription<rm_referee_msgs::msg::RobotStatus>(
        "/rm_referee/robot_status",
        rclcpp::QoS(10),
        [status_prom](const rm_referee_msgs::msg::RobotStatus::SharedPtr msg) {
            try {
                status_prom->set_value(msg);
            } catch (...) {
                // 已设置或其他错误，忽略
            }
        });

    // 等待 robot_status，最多等待 2 秒
    const auto status_timeout = std::chrono::milliseconds(2000);
    const auto status_start = std::chrono::steady_clock::now();
    while (status_fut.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
        rclcpp::spin_some(status_node);
        if (std::chrono::steady_clock::now() - status_start > status_timeout) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    LocSystem::Options opt;
    LocSystem loc(opt);
    if (!loc.Init(FLAGS_config)) {
        LOG(ERROR) << "failed to init loc";
    }

    // 通过 bringup 的 share 目录读取
    std::string kPoseYaml = ament_index_cpp::get_package_share_directory("bringup") + "/config/loc_start_pose.yaml";


    int robot_id = -1;
    if (status_fut.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        auto status_msg = status_fut.get();
        robot_id = static_cast<int>(status_msg->robot_id);
        LOG(INFO) << "收到 /rm_referee/robot_status，robot_id=" << robot_id;
    } else {
        LOG(WARNING) << "未收到 /rm_referee/robot_status，使用默认 red 位姿";
    }

    std::string side = (robot_id == 7) ? "red" : (robot_id == 107) ? "blue" : "red";

    double x = 0.0, y = 0.0, z = 0.0;
    double roll_deg = 0.0, pitch_deg = 0.0, yaw_deg = 0.0;

    try {
        YAML::Node cfg = YAML::LoadFile(kPoseYaml);
        auto pose_node = cfg["start_pose"][side];
        if (pose_node) {
            x = pose_node["x"].as<double>();
            y = pose_node["y"].as<double>();
            z = pose_node["z"].as<double>();
            yaw_deg = pose_node["yaw_deg"].as<double>();
            pitch_deg = pose_node["pitch_deg"].as<double>();
            roll_deg = pose_node["roll_deg"].as<double>();
        } else {
            LOG(WARNING) << "YAML 中未找到 start_pose." << side << "，使用全零位姿";
        }
    } catch (const std::exception& e) {
        LOG(ERROR) << "读取 YAML 失败: " << e.what() << "，使用全零位姿";
    }

    auto deg2rad = [](double d) { return d * M_PI / 180.0; };
    double roll_rad = deg2rad(roll_deg);
    double pitch_rad = deg2rad(pitch_deg);
    double yaw_rad = deg2rad(yaw_deg);

    Eigen::Vector3d translation(x, y, z);
    Sophus::SO3d rotation = Sophus::SO3d::rotZ(yaw_rad) * Sophus::SO3d::rotY(pitch_rad) * Sophus::SO3d::rotX(roll_rad);
    Sophus::SE3d init_pose(rotation, translation);

    std::string kLidarYaml = ament_index_cpp::get_package_share_directory("bringup") + "/config/lidar.yaml";
    double base_link_to_livox_x = 0.16;
    double base_link_to_livox_y = 0.0;
    double base_link_to_livox_z = 0.0;
    double base_link_to_livox_yaw = 0.0;
    double base_link_to_livox_pitch = 0.0;
    double base_link_to_livox_roll = 0.0;

    try {
        YAML::Node lidar_cfg = YAML::LoadFile(kLidarYaml);
        auto tf_paras = lidar_cfg["tf_odom_publisher"]["ros__parameters"];
        if(tf_paras){
            base_link_to_livox_x = tf_paras["base_link_to_livox_x"].as<double>();
            base_link_to_livox_y = tf_paras["base_link_to_livox_y"].as<double>();
            base_link_to_livox_z = tf_paras["base_link_to_livox_z"].as<double>();
            base_link_to_livox_yaw = tf_paras["base_link_to_livox_yaw"].as<double>();
            base_link_to_livox_pitch = tf_paras["base_link_to_livox_pitch"].as<double>();
            base_link_to_livox_roll = tf_paras["base_link_to_livox_roll"].as<double>();
        }
    } catch (const std::exception& e) {
            LOG(ERROR) << "读取lidar.yaml失败: " << e.what() << "，使用默认tf参数";
        }

    double base_link_roll_red = deg2rad(base_link_to_livox_roll);
    double base_link_pitch_red = deg2rad(base_link_to_livox_pitch);
    double base_link_yaw_red = deg2rad(base_link_to_livox_yaw);
    
    Eigen::Vector3d base_link_translation_red(base_link_to_livox_x, base_link_to_livox_y, base_link_to_livox_z);
    Sophus::SO3d base_link_rotation_red = Sophus::SO3d::rotZ(base_link_yaw_red) * Sophus::SO3d::rotY(base_link_pitch_red) * Sophus::SO3d::rotX(base_link_roll_red);
    Sophus::SE3d base_link_to_livox_red(base_link_rotation_red, base_link_translation_red);
    Sophus::SE3d init_pose_final = init_pose * base_link_to_livox_red;

    LOG(INFO) << "使用 " << side << " 初始位姿: t=[" << init_pose_final.translation().transpose()
              << "], rpy_deg=[" << roll_deg << ", " << pitch_deg << ", " << yaw_deg << "]";

    loc.SetInitPose(init_pose_final);

    loc.Spin();

    rclcpp::shutdown();
    return 0;
}
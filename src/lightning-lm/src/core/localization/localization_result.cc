//
// Created by xiang on 24-4-11.
//

#include "localization_result.h"
#include "core/lightning_math.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <std_msgs/msg/float32.hpp>

// 直接在 core 层发布 ROS 话题（按你的要求）
// 要求：时间戳 + 置信度发布到同一个话题，且每产生一次 confidence 就发布一次
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <mutex>
#include <thread>

namespace lightning::loc {

namespace {
// 单一话题：同时承载 timestamp + confidence
// 消息类型：std_msgs/Float64MultiArray
// data[0] = timestamp(sec), data[1] = confidence
constexpr const char* kTopicTimeAndConfidence = "/confidence_lightninglm";
}  // namespace

namespace {

struct DebugRosPub {
    std::mutex mtx;
    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub;

    rclcpp::executors::SingleThreadedExecutor exec;
    bool spinning = false;
    std::thread spin_thread;

    void ensureInit() {
        if (node) return;

        // 未 init 时创建 Node 会抛异常；ok()==false 直接跳过
        if (!rclcpp::ok()) return;

        try {
            node = std::make_shared<rclcpp::Node>("lightning_localization_result_debug_pub");
            auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
            pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(kTopicTimeAndConfidence, qos);

            // 确保该 node 被 spin，graph 才更稳定可见
            if (!spinning) {
                exec.add_node(node);
                spinning = true;
                spin_thread = std::thread([this]() { exec.spin(); });
                spin_thread.detach(); // 进程退出由 rclcpp shutdown 终止
            }
        } catch (...) {
            node.reset();
            pub.reset();
            spinning = false;
        }
    }

    void publish(const double timestamp_sec, const double confidence) {
        std::lock_guard<std::mutex> lk(mtx);
        ensureInit();
        if (!node || !pub) return;

        std_msgs::msg::Float64MultiArray msg;
        msg.data.resize(2);
        msg.data[0] = timestamp_sec;
        msg.data[1] = confidence;
        pub->publish(msg);
    }
};

DebugRosPub& debugPub() {
    static DebugRosPub inst;
    return inst;
}

}  // namespace

builtin_interfaces::msg::Time LocalizationResult::ToRosTimeMsg() const {
    return math::FromSec(timestamp_);
}

std_msgs::msg::Float32 LocalizationResult::ToConfidenceMsg() const {
    // 仍保留：如果有人调用这个函数，也会触发发布
    debugPub().publish(timestamp_, confidence_);

    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(confidence_);
    return msg;
}

geometry_msgs::msg::TransformStamped LocalizationResult::ToGeoMsg() const {
    // 新增：更“每帧必走”的路径里也发布一次，确保“产生一次 confidence 就发布一次”
    debugPub().publish(timestamp_, confidence_);

    geometry_msgs::msg::TransformStamped msg;
    msg.header.frame_id = "map_reloc";
    msg.header.stamp = math::FromSec(timestamp_);
    msg.child_frame_id = "livox_frame_reloc";

    msg.transform.translation.x = pose_.translation().x();
    msg.transform.translation.y = pose_.translation().y();
    msg.transform.translation.z = pose_.translation().z();

    msg.transform.rotation.x = pose_.so3().unit_quaternion().x();
    msg.transform.rotation.y = pose_.so3().unit_quaternion().y();
    msg.transform.rotation.z = pose_.so3().unit_quaternion().z();
    msg.transform.rotation.w = pose_.so3().unit_quaternion().w();

    return msg;
}

// ...existing code...
NavState LocalizationResult::ToNavState() const {
    NavState ret;
    ret.timestamp_ = timestamp_;
    ret.confidence_ = confidence_;
    ret.pos_ = pose_.translation();
    ret.rot_ = pose_.so3();
    ret.pose_is_ok_ = status_ == LocalizationStatus::GOOD;
    ret.vel_ = (pose_.so3() * vel_b_);
    return ret;
}

}  // namespace lightning::loc
#include "sentry_nav_bt_test/publish_vw_action.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sentry_nav_bt_test
{

PublishVw::PublishVw(const std::string &name, const BT::NodeConfig &config)
    : BT::SyncActionNode(name, config)
{
    if (!config.blackboard || !config.blackboard->get("node", node_)) {
        throw std::runtime_error("PublishVw: missing 'node' in blackboard");
    }
    control_publishers_ = std::make_shared<ControlTopicPublishers>(node_);
}

BT::PortsList PublishVw::providedPorts()
{
    return {
        BT::InputPort<float>("value", "发布到 /vw 的小陀螺速度"),
        BT::InputPort<int>("republish_interval_ms", 50, "相同速度的最小重复发布间隔")};
}

BT::NodeStatus PublishVw::tick()
{
    float value = 0.0F;
    int republish_interval_ms = 50;
    if (!getInput("value", value) || !std::isfinite(value)) {
        RCLCPP_ERROR(node_->get_logger(), "PublishVw: value 缺失或不是有限数值");
        return BT::NodeStatus::FAILURE;
    }
    getInput("republish_interval_ms", republish_interval_ms);
    republish_interval_ms = std::max(0, republish_interval_ms);

    const auto now = std::chrono::steady_clock::now();
    const bool value_changed = !has_published_ || value != last_value_;
    const bool interval_elapsed = !has_published_ ||
        now - last_publish_time_ >= std::chrono::milliseconds(republish_interval_ms);
    if (!value_changed && !interval_elapsed) {
        return BT::NodeStatus::SUCCESS;
    }

    if (!control_publishers_->publishVw(value)) {
        return BT::NodeStatus::FAILURE;
    }

    if (value_changed) {
        RCLCPP_INFO(node_->get_logger(), "[VW] 行为树发布 /vw = %.3f", value);
    }
    last_value_ = value;
    last_publish_time_ = now;
    has_published_ = true;
    return BT::NodeStatus::SUCCESS;
}

PublishScanMode::PublishScanMode(const std::string &name, const BT::NodeConfig &config)
    : BT::SyncActionNode(name, config)
{
    if (!config.blackboard || !config.blackboard->get("node", node_)) {
        throw std::runtime_error("PublishScanMode: missing 'node' in blackboard");
    }
    control_publishers_ = std::make_shared<ControlTopicPublishers>(node_);
}

BT::PortsList PublishScanMode::providedPorts()
{
    return {BT::InputPort<bool>("enabled", false, "true 为定向 yaw，false 为旋转扫描")};
}

BT::NodeStatus PublishScanMode::tick()
{
    bool enabled = false;
    getInput("enabled", enabled);
    if (!control_publishers_->publishScanMode(enabled)) {
        return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(node_->get_logger(), "[ScanMode] 发布 /scan_mod_type = %d", enabled ? 1 : 0);
    return BT::NodeStatus::SUCCESS;
}

}  // namespace sentry_nav_bt_test

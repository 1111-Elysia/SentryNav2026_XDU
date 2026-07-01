#include "sentry_nav_bt_test/referee/control_topic_publishers.hpp"

namespace sentry_nav_bt_test
{

ControlTopicPublishers::ControlTopicPublishers(const rclcpp::Node::SharedPtr &node)
    : node_(node)
{
}

bool ControlTopicPublishers::publishVw(float value)
{
    ensureInitialized();
    if (!vw_pub_) {
        logMissingPublisher("/vw");
        return false;
    }
    sentry_msgs::msg::Vw msg;
    msg.vw = value;
    vw_pub_->publish(msg);
    return true;
}

bool ControlTopicPublishers::publishScanMode(bool yaw_control_enabled)
{
    ensureInitialized();
    if (!scan_mode_pub_) {
        logMissingPublisher("/scan_mod_type");
        return false;
    }
    sentry_msgs::msg::ScanMode msg;
    msg.scan_mod_type = yaw_control_enabled;
    scan_mode_pub_->publish(msg);
    return true;
}

bool ControlTopicPublishers::publishAutoShoot(bool enabled)
{
    ensureInitialized();
    if (!auto_shoot_pub_) {
        logMissingPublisher("/auto_shoot_type");
        return false;
    }
    std_msgs::msg::Bool msg;
    msg.data = enabled;
    auto_shoot_pub_->publish(msg);
    return true;
}

bool ControlTopicPublishers::publishYawController(int target)
{
    ensureInitialized();
    if (!yaw_controller_pub_) {
        logMissingPublisher("/yaw_controller");
        return false;
    }
    std_msgs::msg::Int32 msg;
    msg.data = target;
    yaw_controller_pub_->publish(msg);
    return true;
}

bool ControlTopicPublishers::publishOutpostMode(bool enabled)
{
    ensureInitialized();
    if (!outpost_mode_pub_) {
        logMissingPublisher("/outpost_mode_type");
        return false;
    }
    std_msgs::msg::Bool msg;
    msg.data = enabled;
    outpost_mode_pub_->publish(msg);
    return true;
}

void ControlTopicPublishers::ensureInitialized()
{
    if (initialized_ || !node_) {
        return;
    }
    vw_pub_ = node_->create_publisher<sentry_msgs::msg::Vw>("/vw", 10);
    scan_mode_pub_ = node_->create_publisher<sentry_msgs::msg::ScanMode>("/scan_mod_type", 10);
    auto_shoot_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/auto_shoot_type", 10);
    yaw_controller_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/yaw_controller", 10);
    outpost_mode_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/outpost_mode_type", 10);
    initialized_ = true;
}

void ControlTopicPublishers::logMissingPublisher(const char *topic_name) const
{
    const auto logger = node_ ? node_->get_logger() : rclcpp::get_logger("ControlTopicPublishers");
    RCLCPP_ERROR(logger, "未初始化 %s 发布器", topic_name);
}

}  // namespace sentry_nav_bt_test

#ifndef SENTRY_NAV_BT_REFEREE_CONTROL_TOPIC_PUBLISHERS_HPP_
#define SENTRY_NAV_BT_REFEREE_CONTROL_TOPIC_PUBLISHERS_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sentry_msgs/msg/scan_mode.hpp"
#include "sentry_msgs/msg/vw.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"

namespace sentry_nav_bt
{

// 集中持有裁判任务涉及的控制话题，避免每个 BT 动作重复创建发布器。
class ControlTopicPublishers
{
public:
    explicit ControlTopicPublishers(const rclcpp::Node::SharedPtr &node);

    bool publishVw(float value);
    bool publishScanMode(bool yaw_control_enabled);
    bool publishAutoShoot(bool enabled);
    bool publishYawController(int target);
    bool publishOutpostMode(bool enabled);

private:
    void ensureInitialized();
    void logMissingPublisher(const char *topic_name) const;

    rclcpp::Node::SharedPtr node_;
    bool initialized_{false};
    rclcpp::Publisher<sentry_msgs::msg::Vw>::SharedPtr vw_pub_;
    rclcpp::Publisher<sentry_msgs::msg::ScanMode>::SharedPtr scan_mode_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr auto_shoot_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr yaw_controller_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr outpost_mode_pub_;
};

}  // namespace sentry_nav_bt

#endif  // SENTRY_NAV_BT_REFEREE_CONTROL_TOPIC_PUBLISHERS_HPP_

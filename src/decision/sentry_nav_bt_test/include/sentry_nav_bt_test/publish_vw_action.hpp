#ifndef SENTRY_NAV_BT_TEST_PUBLISH_VW_ACTION_HPP_
#define SENTRY_NAV_BT_TEST_PUBLISH_VW_ACTION_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sentry_nav_bt_test/referee/control_topic_publishers.hpp"

namespace sentry_nav_bt_test
{

// 由行为树向小陀螺控制话题发布速度，具体触发条件保留在 XML 中。
class PublishVw : public BT::SyncActionNode
{
public:
    PublishVw(const std::string &name, const BT::NodeConfig &config);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<ControlTopicPublishers> control_publishers_;
    std::chrono::steady_clock::time_point last_publish_time_{};
    float last_value_{0.0F};
    bool has_published_{false};
};

// 复用控制话题发布器，向扫描模式话题发布一次指定状态。
class PublishScanMode : public BT::SyncActionNode
{
public:
    PublishScanMode(const std::string &name, const BT::NodeConfig &config);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<ControlTopicPublishers> control_publishers_;
};

}  // namespace sentry_nav_bt_test

#endif  // SENTRY_NAV_BT_TEST_PUBLISH_VW_ACTION_HPP_

#ifndef SENTRY_NAV_BT_TEST_GOAL_SELECTOR_NODE_HPP_
#define SENTRY_NAV_BT_TEST_GOAL_SELECTOR_NODE_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"

namespace sentry_nav_bt_test
{

class GoalSelector : public BT::SyncActionNode
{
public:
    GoalSelector(
        const std::string &xml_tag_name,
        const BT::NodeConfig &conf);

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("goal_name", "目标点的名称，将在黑板中查找"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal", "输出的目标位置")
        };
    }

    BT::NodeStatus tick() override;

private:
    rclcpp::Logger logger_;
    std::string last_logged_goal_name_;
    geometry_msgs::msg::PoseStamped last_logged_goal_pose_;
    bool has_last_logged_goal_{false};
};

} // namespace sentry_nav_bt_test

#endif // SENTRY_NAV_BT_TEST_GOAL_SELECTOR_NODE_HPP_
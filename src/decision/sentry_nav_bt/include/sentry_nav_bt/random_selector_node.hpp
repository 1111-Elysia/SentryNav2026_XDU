#ifndef SENTRY_NAV_BT_RANDOM_SELECTOR_NODE_HPP_
#define SENTRY_NAV_BT_RANDOM_SELECTOR_NODE_HPP_

#include <string>
#include <vector>
#include <random>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace sentry_nav_bt
{

class RandomSelector : public BT::SyncActionNode
{
public:
  RandomSelector(
    const std::string & xml_tag_name,
    const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("goal_names", "逗号分隔的目标点名称列表"),
      BT::InputPort<bool>("avoid_repeat", true, "是否避免连续选择同一个目标点"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal", "随机选中的目标位置"),
      BT::OutputPort<std::string>("goal_name", "随机选中的目标点名称")
    };
  }

  BT::NodeStatus tick() override;

private:
  std::vector<std::string> parseGoalNames(const std::string &goal_names) const;

  std::random_device rd_;
  std::mt19937 gen_;
  rclcpp::Logger logger_;
  std::string last_goal_name_;
};

}  // namespace sentry_nav_bt

#endif  // SENTRY_NAV_BT_RANDOM_SELECTOR_NODE_HPP_

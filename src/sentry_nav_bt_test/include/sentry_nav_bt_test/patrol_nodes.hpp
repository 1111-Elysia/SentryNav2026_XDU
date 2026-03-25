#ifndef SENTRY_NAV_BT_TEST_PATROL_NODES_HPP_
#define SENTRY_NAV_BT_TEST_PATROL_NODES_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace sentry_nav_bt_test
{

class PatrolGoalSelector : public BT::SyncActionNode
{
public:
  PatrolGoalSelector(const std::string &name, const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  std::vector<std::string> parseGoalNames(const std::string &goal_names) const;

  rclcpp::Logger logger_;
};

class CheckGoalReached : public BT::ConditionNode
{
public:
  CheckGoalReached(const std::string &name, const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Logger logger_;
  geometry_msgs::msg::PoseStamped last_goal_pose_;
  double last_threshold_{-1.0};
  bool has_goal_context_{false};
  bool last_reported_reached_{false};
  bool last_pose_invalid_reported_{false};
};

}  // namespace sentry_nav_bt_test

#endif  // SENTRY_NAV_BT_TEST_PATROL_NODES_HPP_

#ifndef SENTRY_NAV_BT_PATROL_NODES_HPP_
#define SENTRY_NAV_BT_PATROL_NODES_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace sentry_nav_bt
{

class PatrolGoalSelector : public BT::SyncActionNode
{
public:
  PatrolGoalSelector(const std::string &name, const BT::NodeConfig &config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  std::vector<std::string> parseGoalNames(const std::string &goal_names) const;

  rclcpp::Logger logger_;
};

class CheckGoalReached : public BT::ConditionNode
{
public:
  CheckGoalReached(const std::string &name, const BT::NodeConfig &config);

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

// 运行期间启用追击规划器；被上级行为树抢占时恢复普通规划器。
class UseTrackingPlanner : public BT::StatefulActionNode
{
public:
  UseTrackingPlanner(const std::string &name, const BT::NodeConfig &config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  bool publishPlanner(const std::string &planner_name);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::string tracking_planner_;
  std::string fallback_planner_;
};

}  // namespace sentry_nav_bt

#endif  // SENTRY_NAV_BT_PATROL_NODES_HPP_

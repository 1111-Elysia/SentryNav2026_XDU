#ifndef SENTRY_NAV_BT_TEST_PATROL_NODES_HPP_
#define SENTRY_NAV_BT_TEST_PATROL_NODES_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

namespace sentry_nav_bt_test
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

// 目标距离持续有效，或自最后一次有效观测起尚未超过丢失容忍时间。
class TargetDetected : public BT::ConditionNode
{
public:
  TargetDetected(const std::string &name, const BT::NodeConfig &config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void ensureSubscription();
  void distanceCallback(const std_msgs::msg::Float32::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  std::mutex mutex_;
  rclcpp::Time last_valid_time_{0, 0, RCL_ROS_TIME};
  bool has_valid_observation_{false};
  bool last_reported_detected_{false};
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
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

}  // namespace sentry_nav_bt_test

#endif  // SENTRY_NAV_BT_TEST_PATROL_NODES_HPP_

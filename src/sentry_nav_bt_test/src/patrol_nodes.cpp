#include "sentry_nav_bt_test/patrol_nodes.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace sentry_nav_bt_test
{

namespace
{

std::string trim(const std::string &value)
{
  const auto begin = value.find_first_not_of(" \t");
  if (begin == std::string::npos) {
    return "";
  }

  const auto end = value.find_last_not_of(" \t");
  return value.substr(begin, end - begin + 1);
}

}  // namespace

PatrolGoalSelector::PatrolGoalSelector(
  const std::string &name, const BT::NodeConfiguration &config)
: BT::SyncActionNode(name, config),
  logger_(rclcpp::get_logger("PatrolGoalSelector"))
{
}

BT::PortsList PatrolGoalSelector::providedPorts()
{
  return {
    BT::InputPort<std::string>("goal_names", "用逗号分隔的巡逻点名称列表"),
    BT::InputPort<std::string>("index_key", "巡逻索引黑板键", "patrol_index"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal", "输出当前巡逻目标"),
    BT::OutputPort<std::string>("goal_name", "输出当前巡逻点名称")
  };
}

std::vector<std::string> PatrolGoalSelector::parseGoalNames(const std::string &goal_names) const
{
  std::vector<std::string> names;
  std::stringstream ss(goal_names);
  std::string item;

  while (std::getline(ss, item, ',')) {
    auto name = trim(item);
    if (!name.empty()) {
      names.push_back(name);
    }
  }

  return names;
}

BT::NodeStatus PatrolGoalSelector::tick()
{
  auto blackboard = config().blackboard;
  if (!blackboard) {
    RCLCPP_ERROR(logger_, "无法获取黑板");
    return BT::NodeStatus::FAILURE;
  }

  std::string goal_names_raw;
  if (!getInput("goal_names", goal_names_raw)) {
    RCLCPP_ERROR(logger_, "缺少必要参数 'goal_names'");
    return BT::NodeStatus::FAILURE;
  }

  std::string index_key;
  getInput("index_key", index_key);

  const auto goal_names = parseGoalNames(goal_names_raw);
  if (goal_names.empty()) {
    RCLCPP_ERROR(logger_, "'goal_names' 中没有可用的巡逻点");
    return BT::NodeStatus::FAILURE;
  }

  int patrol_index = 0;
  if (!blackboard->get(index_key, patrol_index)) {
    blackboard->set(index_key, 0);
    patrol_index = 0;
  }

  if (patrol_index < 0) {
    patrol_index = 0;
  }

  const size_t normalized_index =
    static_cast<size_t>(patrol_index) % goal_names.size();
  const std::string &selected_name = goal_names[normalized_index];
  const std::string waypoint_key = "waypoint_" + selected_name;

  geometry_msgs::msg::PoseStamped goal_pose;
  if (!blackboard->get(waypoint_key, goal_pose)) {
    RCLCPP_ERROR(
      logger_, "找不到巡逻点 '%s'（黑板键 '%s'）",
      selected_name.c_str(), waypoint_key.c_str());
    return BT::NodeStatus::FAILURE;
  }

  goal_pose.header.stamp = rclcpp::Clock().now();
  setOutput("goal", goal_pose);
  setOutput("goal_name", selected_name);

  blackboard->set(index_key, static_cast<int>((normalized_index + 1) % goal_names.size()));

  RCLCPP_INFO(
    logger_, "巡逻目标切换到 '%s'，位置(%.2f, %.2f)",
    selected_name.c_str(),
    goal_pose.pose.position.x,
    goal_pose.pose.position.y);

  return BT::NodeStatus::SUCCESS;
}

CheckGoalReached::CheckGoalReached(
  const std::string &name, const BT::NodeConfiguration &config)
: BT::ConditionNode(name, config),
  logger_(rclcpp::get_logger("CheckGoalReached"))
{
}

BT::PortsList CheckGoalReached::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "当前目标点"),
    BT::InputPort<double>("threshold", 0.3, "到点距离阈值")
  };
}

BT::NodeStatus CheckGoalReached::tick()
{
  auto blackboard = config().blackboard;
  if (!blackboard) {
    RCLCPP_ERROR(logger_, "无法获取黑板");
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped goal_pose;
  if (!getInput("goal", goal_pose)) {
    RCLCPP_ERROR(logger_, "缺少必要参数 'goal'");
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!blackboard->get("waypoint_now", current_pose)) {
    RCLCPP_WARN(logger_, "黑板中还没有 'waypoint_now'，暂时无法判断是否到点");
    return BT::NodeStatus::FAILURE;
  }

  double threshold = 0.3;
  getInput("threshold", threshold);

  const double dx = goal_pose.pose.position.x - current_pose.pose.position.x;
  const double dy = goal_pose.pose.position.y - current_pose.pose.position.y;
  const double distance = std::hypot(dx, dy);

  const bool goal_changed =
    !has_goal_context_ ||
    std::abs(goal_pose.pose.position.x - last_goal_pose_.pose.position.x) > 1e-3 ||
    std::abs(goal_pose.pose.position.y - last_goal_pose_.pose.position.y) > 1e-3 ||
    std::abs(threshold - last_threshold_) > 1e-6;

  if (goal_changed) {
    last_goal_pose_ = goal_pose;
    last_threshold_ = threshold;
    has_goal_context_ = true;
    last_reported_reached_ = false;
  }

  if (distance <= threshold) {
    if (!last_reported_reached_) {
      RCLCPP_INFO(
        logger_, "已到达目标点附近，当前距离 %.3f m，阈值 %.3f m",
        distance, threshold);
      last_reported_reached_ = true;
    }
    return BT::NodeStatus::SUCCESS;
  }

  last_reported_reached_ = false;
  return BT::NodeStatus::FAILURE;
}

}  // namespace sentry_nav_bt_test

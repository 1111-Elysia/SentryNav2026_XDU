#include "sentry_nav_bt_test/runtime_config.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace sentry_nav_bt_test
{
namespace
{

constexpr char kGoalName[] = "runtime_goal_name";
constexpr char kUseCustomPose[] = "runtime_use_custom_pose";
constexpr char kGoalX[] = "runtime_goal_x";
constexpr char kGoalY[] = "runtime_goal_y";
constexpr char kGoalYaw[] = "runtime_goal_yaw";
constexpr char kMovePosture[] = "runtime_move_posture";
constexpr char kWaitPosture[] = "runtime_wait_posture";
constexpr char kReachThreshold[] = "runtime_reach_threshold";
constexpr char kWaitTimeThreshold[] = "runtime_wait_time_threshold";

bool isRuntimeParameter(const std::string & name)
{
  return name == kGoalName || name == kUseCustomPose || name == kGoalX ||
         name == kGoalY || name == kGoalYaw || name == kMovePosture ||
         name == kWaitPosture ||
         name == kReachThreshold || name == kWaitTimeThreshold;
}

std::string trim(const std::string & value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

}  // namespace

RuntimeConfigManager::RuntimeConfigManager(
  const rclcpp::Node::SharedPtr & node,
  const BT::Blackboard::Ptr & blackboard)
: node_(node), blackboard_(blackboard)
{
  node_->declare_parameter<std::string>(kGoalName, "init");
  node_->declare_parameter<bool>(kUseCustomPose, false);
  node_->declare_parameter<double>(kGoalX, 0.0);
  node_->declare_parameter<double>(kGoalY, 0.0);
  node_->declare_parameter<double>(kGoalYaw, 0.0);
  node_->declare_parameter<int>(kMovePosture, 3);
  node_->declare_parameter<int>(kWaitPosture, 1);
  node_->declare_parameter<double>(kReachThreshold, 0.25);
  node_->declare_parameter<double>(kWaitTimeThreshold, 5.0);

  callback_handle_ = node_->add_on_set_parameters_callback(
    std::bind(&RuntimeConfigManager::onParametersChanged, this, std::placeholders::_1));
}

RuntimeConfigManager::Config RuntimeConfigManager::currentConfig() const
{
  Config config;
  config.goal_name = node_->get_parameter(kGoalName).as_string();
  config.use_custom_pose = node_->get_parameter(kUseCustomPose).as_bool();
  config.goal_x = node_->get_parameter(kGoalX).as_double();
  config.goal_y = node_->get_parameter(kGoalY).as_double();
  config.goal_yaw = node_->get_parameter(kGoalYaw).as_double();
  config.move_posture = static_cast<int>(node_->get_parameter(kMovePosture).as_int());
  config.wait_posture = static_cast<int>(node_->get_parameter(kWaitPosture).as_int());
  config.reach_threshold = node_->get_parameter(kReachThreshold).as_double();
  config.wait_time_threshold = node_->get_parameter(kWaitTimeThreshold).as_double();
  return config;
}

bool RuntimeConfigManager::validate(const Config & config, std::string & reason) const
{
  if (trim(config.goal_name).empty()) {
    reason = "runtime_goal_name 不能为空";
    return false;
  }
  if (!std::isfinite(config.goal_x) || !std::isfinite(config.goal_y) ||
      !std::isfinite(config.goal_yaw))
  {
    reason = "目标坐标和 yaw 必须是有限数值";
    return false;
  }
  if (config.move_posture < 1 || config.move_posture > 6 ||
      config.wait_posture < 1 || config.wait_posture > 6)
  {
    reason = "姿态必须位于 1..6";
    return false;
  }
  if (!std::isfinite(config.reach_threshold) || config.reach_threshold <= 0.0) {
    reason = "runtime_reach_threshold 必须大于 0";
    return false;
  }
  if (!std::isfinite(config.wait_time_threshold) || config.wait_time_threshold < 0.0) {
    reason = "runtime_wait_time_threshold 不能小于 0";
    return false;
  }

  if (!config.use_custom_pose) {
    geometry_msgs::msg::PoseStamped pose;
    if (!blackboard_->get("waypoint_" + trim(config.goal_name), pose)) {
      reason = "目标点不存在: " + trim(config.goal_name);
      return false;
    }
  }
  return true;
}

void RuntimeConfigManager::applyToBlackboard(const Config & config)
{
  std::string effective_goal_name = trim(config.goal_name);
  if (config.use_custom_pose) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = node_->now();
    pose.pose.position.x = config.goal_x;
    pose.pose.position.y = config.goal_y;

    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, config.goal_yaw);
    pose.pose.orientation.x = orientation.x();
    pose.pose.orientation.y = orientation.y();
    pose.pose.orientation.z = orientation.z();
    pose.pose.orientation.w = orientation.w();
    blackboard_->set("waypoint_gui_runtime", pose);
    effective_goal_name = "gui_runtime";
  }

  blackboard_->set("runtime_effective_goal_name", effective_goal_name);
  blackboard_->set("runtime_move_posture", config.move_posture);
  blackboard_->set("runtime_wait_posture", config.wait_posture);
  blackboard_->set("runtime_reach_threshold", config.reach_threshold);
  blackboard_->set("runtime_wait_time_threshold", config.wait_time_threshold);
  blackboard_->set("runtime_use_custom_pose", config.use_custom_pose);
  // 每次 GUI 应用配置都重新激活单点任务；否则时间阈值完成后的锁存会吞掉后续目标。
  blackboard_->set("simple_nav_completed", 0);

  RCLCPP_INFO(
    node_->get_logger(),
    "运行配置已更新: goal=%s%s, move_posture=%d, wait_posture=%d, reach=%.3f, wait_time=%.1f",
    effective_goal_name.c_str(), config.use_custom_pose ? "(custom)" : "",
    config.move_posture, config.wait_posture,
    config.reach_threshold, config.wait_time_threshold);
}

bool RuntimeConfigManager::applyCurrentParameters()
{
  const Config config = currentConfig();
  std::string reason;
  if (!validate(config, reason)) {
    RCLCPP_ERROR(node_->get_logger(), "默认运行配置无效: %s", reason.c_str());
    return false;
  }
  applyToBlackboard(config);
  return true;
}

rcl_interfaces::msg::SetParametersResult RuntimeConfigManager::onParametersChanged(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  Config candidate = currentConfig();
  bool runtime_changed = false;
  try {
    for (const auto & parameter : parameters) {
      const std::string & name = parameter.get_name();
      if (!isRuntimeParameter(name)) {
        continue;
      }
      runtime_changed = true;
      if (name == kGoalName) candidate.goal_name = parameter.as_string();
      else if (name == kUseCustomPose) candidate.use_custom_pose = parameter.as_bool();
      else if (name == kGoalX) candidate.goal_x = parameter.as_double();
      else if (name == kGoalY) candidate.goal_y = parameter.as_double();
      else if (name == kGoalYaw) candidate.goal_yaw = parameter.as_double();
      else if (name == kMovePosture) candidate.move_posture = static_cast<int>(parameter.as_int());
      else if (name == kWaitPosture) candidate.wait_posture = static_cast<int>(parameter.as_int());
      else if (name == kReachThreshold) candidate.reach_threshold = parameter.as_double();
      else if (name == kWaitTimeThreshold) candidate.wait_time_threshold = parameter.as_double();
    }
  } catch (const rclcpp::ParameterTypeException & error) {
    result.successful = false;
    result.reason = std::string("参数类型错误: ") + error.what();
    return result;
  }

  if (!runtime_changed) {
    return result;
  }

  if (!validate(candidate, result.reason)) {
    result.successful = false;
    RCLCPP_WARN(node_->get_logger(), "拒绝运行配置: %s", result.reason.c_str());
    return result;
  }

  applyToBlackboard(candidate);
  result.reason = "配置已生效";
  return result;
}

}  // namespace sentry_nav_bt_test

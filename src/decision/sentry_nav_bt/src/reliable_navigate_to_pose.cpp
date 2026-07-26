#include "sentry_nav_bt/reliable_navigate_to_pose.hpp"

#include <cmath>
#include <limits>

#include "navigation_transition_gate.hpp"
#include "sentry_nav_bt/blackboard_utils.hpp"

namespace sentry_nav_bt
{

ReliableNavigateToPose::ReliableNavigateToPose(
  const std::string &name,
  const BT::NodeConfig &config,
  const rclcpp::Node::SharedPtr &node)
: BT::StatefulActionNode(name, config),
  node_(node)
{
  if (!node_) {
    throw BT::RuntimeError("ReliableNavigateToPose: node is null");
  }
  client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
}

BT::PortsList ReliableNavigateToPose::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
    BT::InputPort<std::string>("goal_name", "", "目标点名称，用于日志"),
    BT::InputPort<double>("reach_threshold", 0.30, "到点距离阈值"),
    BT::InputPort<double>("resend_interval", 0.50, "重发最小间隔"),
    BT::InputPort<double>("response_timeout", 1.50, "等待 Nav2 接收的告警阈值；超出后继续等待且不重发"),
    BT::InputPort<double>("result_retry_delay", 0.50, "结果失败后的重试延迟"),
    BT::InputPort<double>("cancel_confirm_timeout", 1.50, "等待取消确认的超时时间"),
    BT::InputPort<int>("log_throttle_ms", 0, "高频 INFO 日志节流时间；0 表示不节流"),
    BT::InputPort<std::string>("success_condition_key", "", "判定导航成功时需要满足的黑板键"),
    BT::InputPort<std::string>("success_condition_comparison", "eq", "成功条件比较符"),
    BT::InputPort<double>("success_condition_threshold", 1.0, "成功条件阈值")
  };
}

bool ReliableNavigateToPose::refreshGoalInput_(bool *goal_changed)
{
  geometry_msgs::msg::PoseStamped next_goal;
  if (!getInput("goal", next_goal)) {
    RCLCPP_ERROR(node_->get_logger(), "[ReliableNavigate] 缺少输入端口 'goal'");
    return false;
  }

  std::string next_goal_name;
  getInput("goal_name", next_goal_name);

  if (next_goal.header.frame_id.empty()) {
    next_goal.header.frame_id = "map";
  }

  const bool pose_changed =
    std::abs(next_goal.pose.position.x - current_goal_.pose.position.x) > 1e-3 ||
    std::abs(next_goal.pose.position.y - current_goal_.pose.position.y) > 1e-3 ||
    std::abs(next_goal.pose.position.z - current_goal_.pose.position.z) > 1e-3 ||
    std::abs(next_goal.pose.orientation.x - current_goal_.pose.orientation.x) > 1e-3 ||
    std::abs(next_goal.pose.orientation.y - current_goal_.pose.orientation.y) > 1e-3 ||
    std::abs(next_goal.pose.orientation.z - current_goal_.pose.orientation.z) > 1e-3 ||
    std::abs(next_goal.pose.orientation.w - current_goal_.pose.orientation.w) > 1e-3;

  const bool changed =
    next_goal_name != goal_name_ ||
    next_goal.header.frame_id != current_goal_.header.frame_id ||
    pose_changed;

  current_goal_ = next_goal;
  goal_name_ = next_goal_name;

  if (goal_changed) {
    *goal_changed = changed;
  }

  return true;
}

void ReliableNavigateToPose::resetRuntimeState_()
{
  state_ = InternalState::IDLE;
  result_ready_ = false;
  result_success_ = false;
  cancel_requested_ = false;
  response_timeout_reported_ = false;
  goal_handle_.reset();
  send_attempts_ = 0;
  retry_count_ = 0;
  canceled_goal_id_ = 0;
  active_goal_id_ = ++seq_;
  active_send_id_ = 0;
  active_navigation_token_ = 0;
  pending_cancel_reason_.clear();
  awaiting_cancel_confirm_ = false;
  success_pending_after_cancel_ = false;
  wait_cancel_goal_id_ = 0;
  const auto clock_type = node_->get_clock()->get_clock_type();
  cancel_wait_start_ = rclcpp::Time(0, 0, clock_type);
  last_send_time_ = rclcpp::Time(0, 0, clock_type);
  last_log_time_ = rclcpp::Time(0, 0, clock_type);
  last_pose_invalid_logged_ = false;
}

BT::NodeStatus ReliableNavigateToPose::onStart()
{
  if (!refreshGoalInput_()) {
    return BT::NodeStatus::FAILURE;
  }

  getInput("reach_threshold", reach_threshold_);
  getInput("resend_interval", resend_interval_);
  getInput("response_timeout", response_timeout_);
  getInput("result_retry_delay", result_retry_delay_);
  getInput("cancel_confirm_timeout", cancel_confirm_timeout_);
  getInput("log_throttle_ms", log_throttle_ms_);
  getInput("success_condition_key", success_condition_key_);
  getInput("success_condition_comparison", success_condition_comparison_);
  getInput("success_condition_threshold", success_condition_threshold_);

  resetRuntimeState_();

  RCLCPP_INFO_THROTTLE(
    node_->get_logger(),
    *node_->get_clock(),
    log_throttle_ms_,
    "[ReliableNavigate] 开始导航%s%s%s -> (%.2f, %.2f), threshold=%.2f",
    goal_name_.empty() ? "" : "[",
    goal_name_.empty() ? "" : goal_name_.c_str(),
    goal_name_.empty() ? "" : "]",
    current_goal_.pose.position.x,
    current_goal_.pose.position.y,
    reach_threshold_);
  if (!goal_name_.empty()) {
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), log_throttle_ms_,
      "[ReliableNavigate] 目标名称: %s", goal_name_.c_str());
  }

  return BT::NodeStatus::RUNNING;
}

bool ReliableNavigateToPose::isSuccessConditionSatisfied_(double *current_value) const
{
  if (success_condition_key_.empty()) {
    return true;
  }

  auto blackboard = config().blackboard;
  if (!blackboard) {
    return false;
  }

  double value = 0.0;
  if (!blackboard_utils::getValue(
      blackboard, success_condition_key_, value, "ReliableNavigateToPose"))
  {
    return false;
  }

  if (current_value) {
    *current_value = value;
  }

  return blackboard_utils::compareValues(
    value,
    success_condition_comparison_,
    success_condition_threshold_,
    "ReliableNavigateToPose");
}

ReliableNavigateToPose::GoalStatus ReliableNavigateToPose::evaluateGoalStatus_(double *distance_out)
{
  if (distance_out) {
    *distance_out = std::numeric_limits<double>::infinity();
  }

  auto blackboard = config().blackboard;
  if (!blackboard) {
    return GoalStatus::NOT_REACHED;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  bool current_pose_valid = false;
  if (!blackboard->get("waypoint_now_valid", current_pose_valid) || !current_pose_valid) {
    if (!last_pose_invalid_logged_) {
      RCLCPP_WARN(node_->get_logger(), "[ReliableNavigate] 当前位姿没有更新，暂不使用到点判定");
      last_pose_invalid_logged_ = true;
    }
    return GoalStatus::NOT_REACHED;
  }

  if (!blackboard->get("waypoint_now", current_pose)) {
    return GoalStatus::NOT_REACHED;
  }

  last_pose_invalid_logged_ = false;

  const double dx = current_goal_.pose.position.x - current_pose.pose.position.x;
  const double dy = current_goal_.pose.position.y - current_pose.pose.position.y;
  const double distance = std::hypot(dx, dy);
  if (distance_out) {
    *distance_out = distance;
  }

  if (distance <= reach_threshold_) {
    double success_condition_value = 0.0;
    if (!isSuccessConditionSatisfied_(&success_condition_value)) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        1000,
        "[ReliableNavigate] 已到达目标%s%s%s附近，但成功条件 '%s' 未满足 (当前=%.3f, 期望 %s %.3f)，继续重发",
        goal_name_.empty() ? "" : "[",
        goal_name_.empty() ? "" : goal_name_.c_str(),
        goal_name_.empty() ? "" : "]",
        success_condition_key_.c_str(),
        success_condition_value,
        success_condition_comparison_.c_str(),
        success_condition_threshold_);
      return GoalStatus::REACHED_GUARD_UNSATISFIED;
    }

    RCLCPP_INFO_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      log_throttle_ms_,
      "[ReliableNavigate] 到达目标%s%s%s, 距离 %.3f m <= %.3f m",
      goal_name_.empty() ? "" : "[",
      goal_name_.empty() ? "" : goal_name_.c_str(),
      goal_name_.empty() ? "" : "]",
      distance,
      reach_threshold_);
    return GoalStatus::REACHED;
  }

  const auto now = node_->now();
  if (last_log_time_.nanoseconds() == 0 || (now - last_log_time_).seconds() > 1.0) {
    RCLCPP_INFO(
      node_->get_logger(),
      "[ReliableNavigate] 导航中%s%s%s, 当前距离 %.3f m",
      goal_name_.empty() ? "" : "[",
      goal_name_.empty() ? "" : goal_name_.c_str(),
      goal_name_.empty() ? "" : "]",
      distance);
    last_log_time_ = now;
  }

  return GoalStatus::NOT_REACHED;
}

BT::NodeStatus ReliableNavigateToPose::onRunning()
{
  // 跨实例门控：旧子树 onHalted() 后，必须等对应 Nav2 goal 到达终态，
  // 才允许新子树发送目标。超时只作为 action 回调永久丢失时的保险。
  bool global_gate_timed_out = false;
  uint64_t pending_navigation_token = 0;
  double global_gate_elapsed_s = 0.0;
  if (navigation_transition::cancelPending(
      cancel_confirm_timeout_,
      &global_gate_timed_out,
      &pending_navigation_token,
      &global_gate_elapsed_s))
  {
    return BT::NodeStatus::RUNNING;
  }
  if (global_gate_timed_out) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[ReliableNavigate] 跨实例等待旧导航取消确认超时(%.2fs, token=%lu)，解除门控",
      global_gate_elapsed_s,
      pending_navigation_token);
  }

  // ── 取消确认门控：上一目标的取消尚未被 Nav2 确认前不做任何事 ──
  // 若此时就放行（返回 SUCCESS / 重发新目标），下一目标会在 bt_navigator
  // 处成为 pending goal，而 SimpleActionServer 在存在 pending goal 时
  // 会屏蔽当前目标的取消请求 → 行为树不重启、旧路径终点被判定到达 → 跳点
  if (awaiting_cancel_confirm_) {
    const auto wait_now = node_->now();
    if ((wait_now - cancel_wait_start_).seconds() < cancel_confirm_timeout_) {
      return BT::NodeStatus::RUNNING;
    }
    RCLCPP_WARN(
      node_->get_logger(),
      "[ReliableNavigate] 等待取消确认超时(%.2fs)%s%s%s, 不再等待",
      cancel_confirm_timeout_,
      goal_name_.empty() ? "" : "[",
      goal_name_.empty() ? "" : goal_name_.c_str(),
      goal_name_.empty() ? "" : "]");
    awaiting_cancel_confirm_ = false;
  }

  if (success_pending_after_cancel_) {
    success_pending_after_cancel_ = false;
    return BT::NodeStatus::SUCCESS;
  }

  geometry_msgs::msg::PoseStamped previous_goal = current_goal_;
  const std::string previous_goal_name = goal_name_;
  bool goal_changed = false;

  if (!refreshGoalInput_(&goal_changed)) {
    return BT::NodeStatus::FAILURE;
  }

  if (goal_changed) {
    RCLCPP_INFO(
      node_->get_logger(),
      "[ReliableNavigate] 检测到目标切换: %s(%.2f, %.2f) -> %s(%.2f, %.2f)",
      previous_goal_name.empty() ? "<unnamed>" : previous_goal_name.c_str(),
      previous_goal.pose.position.x,
      previous_goal.pose.position.y,
      goal_name_.empty() ? "<unnamed>" : goal_name_.c_str(),
      current_goal_.pose.position.x,
      current_goal_.pose.position.y);
    const bool inflight = goal_handle_ || state_ == InternalState::SENDING;
    cancelGoal_("goal_updated");
    const uint64_t canceled_id = canceled_goal_id_;
    resetRuntimeState_();
    if (inflight) {
      // 等旧目标在 Nav2 端确认终止后，再由 IDLE 分支发送新目标
      beginCancelWait_(canceled_id, false);
      return BT::NodeStatus::RUNNING;
    }
  }

  const auto now = node_->now();
  double current_distance = std::numeric_limits<double>::infinity();
  const GoalStatus goal_status = evaluateGoalStatus_(&current_distance);
  if (goal_status == GoalStatus::REACHED) {
    const bool inflight = goal_handle_ || state_ == InternalState::SENDING;
    if (inflight) {
      cancelGoal_("goal_reached_locally");
      // 取消已发出但未确认：等 Nav2 端终止后再返回 SUCCESS，
      // 否则父序列会立刻发下一目标，与本次取消交叠触发跳点
      beginCancelWait_(canceled_goal_id_, true);
      return BT::NodeStatus::RUNNING;
    }
    return BT::NodeStatus::SUCCESS;
  }

  if (goal_status == GoalStatus::REACHED_GUARD_UNSATISFIED &&
    !result_ready_ &&
    !cancel_requested_ &&
    (goal_handle_ || state_ != InternalState::IDLE))
  {
    cancelGoal_("success_condition_unsatisfied");
  }

  if (!client_->action_server_is_ready()) {
    if (last_log_time_.nanoseconds() == 0 || (now - last_log_time_).seconds() > 1.0) {
      RCLCPP_WARN(node_->get_logger(), "[ReliableNavigate] Nav2 action server 尚未就绪，继续等待");
      last_log_time_ = now;
    }
    return BT::NodeStatus::RUNNING;
  }

  if (result_ready_) {
    if (result_success_) {
      // Nav2 的 SUCCEEDED 可能来自抢占切换前的旧路径。只有本节点基于
      // waypoint_now 验证距离和业务成功条件均通过，才允许向父树返回 SUCCESS。
      result_ready_ = false;
      result_success_ = false;
      state_ = InternalState::IDLE;
      goal_handle_.reset();
      ++retry_count_;
      if (std::isfinite(current_distance)) {
        RCLCPP_WARN_THROTTLE(
          node_->get_logger(),
          *node_->get_clock(),
          1000,
          "[ReliableNavigate] Nav2 返回成功%s%s%s，但本地距离 %.3f m > %.3f m，拒绝假成功并准备重发, retry_count=%d",
          goal_name_.empty() ? "" : "[",
          goal_name_.empty() ? "" : goal_name_.c_str(),
          goal_name_.empty() ? "" : "]",
          current_distance,
          reach_threshold_,
          retry_count_);
      } else {
        RCLCPP_WARN_THROTTLE(
          node_->get_logger(),
          *node_->get_clock(),
          1000,
          "[ReliableNavigate] Nav2 返回成功%s%s%s，但当前位姿无效，拒绝假成功并准备重发, retry_count=%d",
          goal_name_.empty() ? "" : "[",
          goal_name_.empty() ? "" : goal_name_.c_str(),
          goal_name_.empty() ? "" : "]",
          retry_count_);
      }
      if ((now - last_send_time_).seconds() < result_retry_delay_) {
        return BT::NodeStatus::RUNNING;
      }
    }

    result_ready_ = false;
    result_success_ = false;
    if ((now - last_send_time_).seconds() < result_retry_delay_) {
      return BT::NodeStatus::RUNNING;
    }
  }

  switch (state_) {
    case InternalState::IDLE:
      if (!cancel_requested_ && (last_send_time_.nanoseconds() == 0 ||
        (now - last_send_time_).seconds() >= resend_interval_)
      ) {
        sendGoal_();
      }
      break;

    case InternalState::SENDING:
      if (!response_timeout_reported_ &&
        (now - last_send_time_).seconds() >= response_timeout_)
      {
        response_timeout_reported_ = true;
        RCLCPP_WARN(
          node_->get_logger(),
          "[ReliableNavigate] 等待 Nav2 接收超过 %.2fs%s%s%s；为避免产生 preemption，保持等待且不重发",
          response_timeout_,
          goal_name_.empty() ? "" : "[",
          goal_name_.empty() ? "" : goal_name_.c_str(),
          goal_name_.empty() ? "" : "]");
      }
      break;

    case InternalState::ACCEPTED:
      if ((now - last_send_time_).seconds() >= response_timeout_ + resend_interval_) {
        RCLCPP_INFO(
          node_->get_logger(),
          "[ReliableNavigate] 目标已受理%s%s%s，继续等待到点/结果",
          goal_name_.empty() ? "" : "[",
          goal_name_.empty() ? "" : goal_name_.c_str(),
          goal_name_.empty() ? "" : "]");
        last_send_time_ = now - rclcpp::Duration::from_seconds(response_timeout_);
      }
      break;
  }

  return BT::NodeStatus::RUNNING;
}

void ReliableNavigateToPose::onHalted()
{
  cancelGoal_("tree_halted");
}

}  // namespace sentry_nav_bt

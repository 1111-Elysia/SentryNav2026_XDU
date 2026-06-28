#include "sentry_nav_bt_test/reliable_navigate_to_pose.hpp"

#include <cmath>
#include <exception>

#include "sentry_nav_bt_test/blackboard_utils.hpp"

namespace sentry_nav_bt_test
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
    BT::InputPort<double>("response_timeout", 1.00, "等待 Nav2 接收超时"),
    BT::InputPort<double>("result_retry_delay", 0.50, "结果失败后的重试延迟"),
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
  goal_handle_.reset();
  send_attempts_ = 0;
  retry_count_ = 0;
  canceled_goal_id_ = 0;
  active_goal_id_ = ++seq_;
  active_send_id_ = 0;
  pending_cancel_reason_.clear();
  const auto clock_type = node_->get_clock()->get_clock_type();
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

ReliableNavigateToPose::GoalStatus ReliableNavigateToPose::evaluateGoalStatus_()
{
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

void ReliableNavigateToPose::sendGoal_()
{
  const auto now = node_->now();
  current_goal_.header.stamp = now;
  const uint64_t goal_id = active_goal_id_;
  const uint64_t send_id = ++send_seq_;
  active_send_id_ = send_id;
  ++send_attempts_;

  NavigateToPose::Goal goal_msg;
  goal_msg.pose = current_goal_;

  auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  options.goal_response_callback =
    [this, goal_id, send_id](const rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr &handle)
    {
      if (goal_id != active_goal_id_) {
        if (handle) {
          tryCancelGoal_(handle, "stale_goal_response");
        }
        RCLCPP_DEBUG(
          node_->get_logger(),
          "[ReliableNavigate] 忽略过期 goal_response, goal_id=%lu current=%lu",
          goal_id,
          active_goal_id_);
        return;
      }

      if (send_id != active_send_id_) {
        if (handle) {
          tryCancelGoal_(handle, "stale_send_response");
          RCLCPP_DEBUG(
            node_->get_logger(),
            "[ReliableNavigate] 取消过期已接收目标%s%s%s, send_id=%lu current=%lu",
            goal_name_.empty() ? "" : "[",
            goal_name_.empty() ? "" : goal_name_.c_str(),
            goal_name_.empty() ? "" : "]",
            send_id,
            active_send_id_);
        }
        return;
      }

      if (!handle) {
        state_ = InternalState::IDLE;
        ++retry_count_;
        RCLCPP_WARN(
          node_->get_logger(),
          "[ReliableNavigate] Nav2 未接收目标%s%s%s, retry_count=%d",
          goal_name_.empty() ? "" : "[",
          goal_name_.empty() ? "" : goal_name_.c_str(),
          goal_name_.empty() ? "" : "]",
          retry_count_);
        return;
      }

      if (cancel_requested_) {
        canceled_goal_id_ = goal_id;
        if (!tryCancelGoal_(
            handle,
            pending_cancel_reason_.empty() ? "cancel_requested" : pending_cancel_reason_.c_str()))
        {
          cancel_requested_ = false;
        }
        RCLCPP_INFO(
          node_->get_logger(),
          "[ReliableNavigate] 目标已接收但当前请求已取消%s%s%s, reason=%s",
          goal_name_.empty() ? "" : "[",
          goal_name_.empty() ? "" : goal_name_.c_str(),
          goal_name_.empty() ? "" : "]",
          pending_cancel_reason_.empty() ? "cancel_requested" : pending_cancel_reason_.c_str());
        return;
      }

      goal_handle_ = handle;
      state_ = InternalState::ACCEPTED;
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        log_throttle_ms_,
        "[ReliableNavigate] Nav2 已接收目标%s%s%s, send_attempt=%d",
        goal_name_.empty() ? "" : "[",
        goal_name_.empty() ? "" : goal_name_.c_str(),
        goal_name_.empty() ? "" : "]",
        send_attempts_);
    };

  options.result_callback =
    [this, goal_id, send_id](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &result)
    {
      if (goal_id != active_goal_id_) {
        RCLCPP_DEBUG(
          node_->get_logger(),
          "[ReliableNavigate] 忽略过期 result, goal_id=%lu current=%lu",
          goal_id,
          active_goal_id_);
        return;
      }

      if (send_id != active_send_id_) {
        RCLCPP_DEBUG(
          node_->get_logger(),
          "[ReliableNavigate] 忽略过期 result callback%s%s%s, send_id=%lu current=%lu",
          goal_name_.empty() ? "" : "[",
          goal_name_.empty() ? "" : goal_name_.c_str(),
          goal_name_.empty() ? "" : "]",
          send_id,
          active_send_id_);
        return;
      }

      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          result_success_ = true;
          result_ready_ = true;
          RCLCPP_INFO_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            log_throttle_ms_,
            "[ReliableNavigate] Nav2 返回成功%s%s%s",
            goal_name_.empty() ? "" : "[",
            goal_name_.empty() ? "" : goal_name_.c_str(),
            goal_name_.empty() ? "" : "]");
          break;

        case rclcpp_action::ResultCode::ABORTED:
          ++retry_count_;
          state_ = InternalState::IDLE;
          goal_handle_.reset();
          result_ready_ = false;
          result_success_ = false;
          RCLCPP_WARN(
            node_->get_logger(),
            "[ReliableNavigate] Nav2 aborted%s%s%s, retry_count=%d",
            goal_name_.empty() ? "" : "[",
            goal_name_.empty() ? "" : goal_name_.c_str(),
            goal_name_.empty() ? "" : "]",
            retry_count_);
          break;

        case rclcpp_action::ResultCode::CANCELED:
          if (goal_id == canceled_goal_id_) {
            cancel_requested_ = false;
            goal_handle_.reset();
            state_ = InternalState::IDLE;
            result_ready_ = true;
            result_success_ = false;
            RCLCPP_INFO(
              node_->get_logger(),
              "[ReliableNavigate] 目标被本节点取消%s%s%s, reason=%s",
              goal_name_.empty() ? "" : "[",
              goal_name_.empty() ? "" : goal_name_.c_str(),
              goal_name_.empty() ? "" : "]",
              pending_cancel_reason_.empty() ? "cancel_requested" : pending_cancel_reason_.c_str());
          } else {
            ++retry_count_;
            state_ = InternalState::IDLE;
            goal_handle_.reset();
            result_ready_ = false;
            result_success_ = false;
            RCLCPP_WARN(
              node_->get_logger(),
              "[ReliableNavigate] 目标被外部取消%s%s%s, retry_count=%d",
              goal_name_.empty() ? "" : "[",
              goal_name_.empty() ? "" : goal_name_.c_str(),
              goal_name_.empty() ? "" : "]",
              retry_count_);
          }
          break;

        default:
          ++retry_count_;
          state_ = InternalState::IDLE;
          goal_handle_.reset();
          result_ready_ = false;
          result_success_ = false;
          RCLCPP_WARN(
            node_->get_logger(),
            "[ReliableNavigate] 未知结果码%s%s%s, retry_count=%d",
            goal_name_.empty() ? "" : "[",
            goal_name_.empty() ? "" : goal_name_.c_str(),
            goal_name_.empty() ? "" : "]",
            retry_count_);
          break;
      }
    };

  client_->async_send_goal(goal_msg, options);
  state_ = InternalState::SENDING;
  last_send_time_ = now;

  RCLCPP_INFO_THROTTLE(
    node_->get_logger(),
    *node_->get_clock(),
    log_throttle_ms_,
    "[ReliableNavigate] 发送目标%s%s%s -> (%.2f, %.2f), attempt=%d, goal_id=%lu",
    goal_name_.empty() ? "" : "[",
    goal_name_.empty() ? "" : goal_name_.c_str(),
    goal_name_.empty() ? "" : "]",
    current_goal_.pose.position.x,
    current_goal_.pose.position.y,
    send_attempts_,
    goal_id);
}

bool ReliableNavigateToPose::tryCancelGoal_(
  const rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr &handle,
  const char *reason)
{
  if (!client_ || !handle) {
    return false;
  }

  try {
    client_->async_cancel_goal(handle);
    return true;
  } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &ex) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[ReliableNavigate] 取消目标时 goal handle 已失效%s%s%s, reason=%s, detail=%s",
      goal_name_.empty() ? "" : "[",
      goal_name_.empty() ? "" : goal_name_.c_str(),
      goal_name_.empty() ? "" : "]",
      reason ? reason : "",
      ex.what());
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[ReliableNavigate] 取消目标失败%s%s%s, reason=%s, detail=%s",
      goal_name_.empty() ? "" : "[",
      goal_name_.empty() ? "" : goal_name_.c_str(),
      goal_name_.empty() ? "" : "]",
      reason ? reason : "",
      ex.what());
  }

  return false;
}

void ReliableNavigateToPose::cancelGoal_(const char *reason)
{
  canceled_goal_id_ = active_goal_id_;
  cancel_requested_ = true;
  pending_cancel_reason_ = reason ? reason : "";

  if (client_ && goal_handle_) {
    if (!tryCancelGoal_(goal_handle_, reason)) {
      cancel_requested_ = false;
    }
    RCLCPP_WARN(
      node_->get_logger(),
      "[ReliableNavigate] 取消当前目标%s%s%s, reason=%s",
      goal_name_.empty() ? "" : "[",
      goal_name_.empty() ? "" : goal_name_.c_str(),
      goal_name_.empty() ? "" : "]",
      reason);
  } else if (reason && *reason != '\0') {
    RCLCPP_INFO(
      node_->get_logger(),
      "[ReliableNavigate] 标记当前目标取消%s%s%s, 等待后续回调处理, reason=%s",
      goal_name_.empty() ? "" : "[",
      goal_name_.empty() ? "" : goal_name_.c_str(),
      goal_name_.empty() ? "" : "]",
      reason);
  }
  goal_handle_.reset();
  state_ = InternalState::IDLE;
}

BT::NodeStatus ReliableNavigateToPose::onRunning()
{
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
    cancelGoal_("goal_updated");
    resetRuntimeState_();
  }

  const auto now = node_->now();
  const GoalStatus goal_status = evaluateGoalStatus_();
  if (goal_status == GoalStatus::REACHED) {
    if (goal_handle_ || state_ != InternalState::IDLE || last_send_time_.nanoseconds() != 0) {
      cancelGoal_("goal_reached_locally");
    }
    return BT::NodeStatus::SUCCESS;
  }

  if (goal_status == GoalStatus::REACHED_GUARD_UNSATISFIED &&
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
      double success_condition_value = 0.0;
      if (isSuccessConditionSatisfied_(&success_condition_value)) {
        return BT::NodeStatus::SUCCESS;
      }

      result_ready_ = false;
      result_success_ = false;
      state_ = InternalState::IDLE;
      goal_handle_.reset();
      ++retry_count_;
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        1000,
        "[ReliableNavigate] Nav2 返回成功%s%s%s，但成功条件 '%s' 未满足 (当前=%.3f, 期望 %s %.3f)，准备重发, retry_count=%d",
        goal_name_.empty() ? "" : "[",
        goal_name_.empty() ? "" : goal_name_.c_str(),
        goal_name_.empty() ? "" : "]",
        success_condition_key_.c_str(),
        success_condition_value,
        success_condition_comparison_.c_str(),
        success_condition_threshold_,
        retry_count_);
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
      if ((now - last_send_time_).seconds() >= response_timeout_) {
        ++retry_count_;
        state_ = InternalState::IDLE;
        RCLCPP_WARN(
          node_->get_logger(),
          "[ReliableNavigate] 等待 Nav2 接收超时，准备重发%s%s%s, retry_count=%d",
          goal_name_.empty() ? "" : "[",
          goal_name_.empty() ? "" : goal_name_.c_str(),
          goal_name_.empty() ? "" : "]",
          retry_count_);
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

}  // namespace sentry_nav_bt_test

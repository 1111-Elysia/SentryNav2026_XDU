#include "sentry_nav_bt_test/reliable_navigate_to_pose.hpp"

#include <cmath>

namespace sentry_nav_bt_test
{

ReliableNavigateToPose::ReliableNavigateToPose(
  const std::string &name,
  const BT::NodeConfiguration &config,
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
    BT::InputPort<double>("result_retry_delay", 0.50, "结果失败后的重试延迟")
  };
}

void ReliableNavigateToPose::resetRuntimeState_()
{
  state_ = InternalState::IDLE;
  result_ready_ = false;
  result_success_ = false;
  goal_handle_.reset();
  send_attempts_ = 0;
  retry_count_ = 0;
  canceled_goal_id_ = 0;
  active_goal_id_ = ++seq_;
  const auto clock_type = node_->get_clock()->get_clock_type();
  last_send_time_ = rclcpp::Time(0, 0, clock_type);
  last_log_time_ = rclcpp::Time(0, 0, clock_type);
}

BT::NodeStatus ReliableNavigateToPose::onStart()
{
  if (!getInput("goal", current_goal_)) {
    RCLCPP_ERROR(node_->get_logger(), "[ReliableNavigate] 缺少输入端口 'goal'");
    return BT::NodeStatus::FAILURE;
  }

  getInput("goal_name", goal_name_);
  getInput("reach_threshold", reach_threshold_);
  getInput("resend_interval", resend_interval_);
  getInput("response_timeout", response_timeout_);
  getInput("result_retry_delay", result_retry_delay_);

  if (current_goal_.header.frame_id.empty()) {
    current_goal_.header.frame_id = "map";
  }

  resetRuntimeState_();

  RCLCPP_INFO(
    node_->get_logger(),
    "[ReliableNavigate] 开始导航%s%s%s -> (%.2f, %.2f), threshold=%.2f",
    goal_name_.empty() ? "" : "[",
    goal_name_.empty() ? "" : goal_name_.c_str(),
    goal_name_.empty() ? "" : "]",
    current_goal_.pose.position.x,
    current_goal_.pose.position.y,
    reach_threshold_);
  if (!goal_name_.empty()) {
    RCLCPP_INFO(node_->get_logger(), "[ReliableNavigate] 目标名称: %s", goal_name_.c_str());
  }

  return BT::NodeStatus::RUNNING;
}

bool ReliableNavigateToPose::isGoalReached_() const
{
  auto blackboard = config().blackboard;
  if (!blackboard) {
    return false;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!blackboard->get("waypoint_now", current_pose)) {
    return false;
  }

  const double dx = current_goal_.pose.position.x - current_pose.pose.position.x;
  const double dy = current_goal_.pose.position.y - current_pose.pose.position.y;
  const double distance = std::hypot(dx, dy);

  if (distance <= reach_threshold_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "[ReliableNavigate] 到达目标%s%s%s, 距离 %.3f m <= %.3f m",
      goal_name_.empty() ? "" : "[",
      goal_name_.empty() ? "" : goal_name_.c_str(),
      goal_name_.empty() ? "" : "]",
      distance,
      reach_threshold_);
    return true;
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
    const_cast<ReliableNavigateToPose *>(this)->last_log_time_ = now;
  }

  return false;
}

void ReliableNavigateToPose::sendGoal_()
{
  const auto now = node_->now();
  current_goal_.header.stamp = now;
  const uint64_t goal_id = active_goal_id_;
  ++send_attempts_;

  NavigateToPose::Goal goal_msg;
  goal_msg.pose = current_goal_;

  auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  options.goal_response_callback =
    [this, goal_id](const rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr &handle)
    {
      if (goal_id != active_goal_id_) {
        RCLCPP_WARN(
          node_->get_logger(),
          "[ReliableNavigate] 忽略过期 goal_response, goal_id=%lu current=%lu",
          goal_id,
          active_goal_id_);
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

      goal_handle_ = handle;
      state_ = InternalState::ACCEPTED;
      RCLCPP_INFO(
        node_->get_logger(),
        "[ReliableNavigate] Nav2 已接收目标%s%s%s, send_attempt=%d",
        goal_name_.empty() ? "" : "[",
        goal_name_.empty() ? "" : goal_name_.c_str(),
        goal_name_.empty() ? "" : "]",
        send_attempts_);
    };

  options.result_callback =
    [this, goal_id](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &result)
    {
      if (goal_id != active_goal_id_) {
        RCLCPP_WARN(
          node_->get_logger(),
          "[ReliableNavigate] 忽略过期 result, goal_id=%lu current=%lu",
          goal_id,
          active_goal_id_);
        return;
      }

      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          result_success_ = true;
          result_ready_ = true;
          RCLCPP_INFO(
            node_->get_logger(),
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
            result_ready_ = true;
            result_success_ = false;
            RCLCPP_INFO(
              node_->get_logger(),
              "[ReliableNavigate] 目标被本节点取消%s%s%s",
              goal_name_.empty() ? "" : "[",
              goal_name_.empty() ? "" : goal_name_.c_str(),
              goal_name_.empty() ? "" : "]");
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

  RCLCPP_INFO(
    node_->get_logger(),
    "[ReliableNavigate] 发送目标%s%s%s -> (%.2f, %.2f), attempt=%d, goal_id=%lu",
    goal_name_.empty() ? "" : "[",
    goal_name_.empty() ? "" : goal_name_.c_str(),
    goal_name_.empty() ? "" : "]",
    current_goal_.pose.position.x,
    current_goal_.pose.position.y,
    send_attempts_,
    goal_id);
}

void ReliableNavigateToPose::cancelGoal_(const char *reason)
{
  if (client_ && goal_handle_) {
    canceled_goal_id_ = active_goal_id_;
    client_->async_cancel_goal(goal_handle_);
    RCLCPP_WARN(
      node_->get_logger(),
      "[ReliableNavigate] 取消当前目标%s%s%s, reason=%s",
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
  if (isGoalReached_()) {
    return BT::NodeStatus::SUCCESS;
  }

  const auto now = node_->now();
  if (!client_->action_server_is_ready()) {
    if (last_log_time_.nanoseconds() == 0 || (now - last_log_time_).seconds() > 1.0) {
      RCLCPP_WARN(node_->get_logger(), "[ReliableNavigate] Nav2 action server 尚未就绪，继续等待");
      last_log_time_ = now;
    }
    return BT::NodeStatus::RUNNING;
  }

  if (result_ready_) {
    if (result_success_) {
      return BT::NodeStatus::SUCCESS;
    }

    result_ready_ = false;
    result_success_ = false;
    if ((now - last_send_time_).seconds() < result_retry_delay_) {
      return BT::NodeStatus::RUNNING;
    }
  }

  switch (state_) {
    case InternalState::IDLE:
      if (last_send_time_.nanoseconds() == 0 ||
        (now - last_send_time_).seconds() >= resend_interval_)
      {
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

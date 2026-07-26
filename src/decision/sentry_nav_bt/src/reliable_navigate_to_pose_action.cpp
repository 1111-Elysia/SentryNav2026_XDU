#include "sentry_nav_bt/reliable_navigate_to_pose.hpp"

#include <exception>

#include "navigation_transition_gate.hpp"

namespace sentry_nav_bt
{

void ReliableNavigateToPose::sendGoal_()
{
  const auto now = node_->now();
  current_goal_.header.stamp = now;
  const uint64_t goal_id = active_goal_id_;
  const uint64_t send_id = ++send_seq_;
  const uint64_t navigation_token = navigation_transition::allocateToken();
  active_send_id_ = send_id;
  active_navigation_token_ = navigation_token;
  response_timeout_reported_ = false;
  ++send_attempts_;

  NavigateToPose::Goal goal_msg;
  goal_msg.pose = current_goal_;

  auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  options.goal_response_callback =
    [this, goal_id, send_id, navigation_token](
      const rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr &handle)
    {
      response_timeout_reported_ = false;

      if (!handle && awaiting_cancel_confirm_ && goal_id == wait_cancel_goal_id_) {
        awaiting_cancel_confirm_ = false;
      }
      if (!handle) {
        navigation_transition::clearCancelPending(navigation_token);
      }

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
    [this, goal_id, send_id, navigation_token](
      const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &result)
    {
      navigation_transition::clearCancelPending(navigation_token);

      if (awaiting_cancel_confirm_ && goal_id == wait_cancel_goal_id_) {
        awaiting_cancel_confirm_ = false;
      }
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
          state_ = InternalState::IDLE;
          goal_handle_.reset();
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
  const bool inflight =
    state_ == InternalState::SENDING ||
    state_ == InternalState::ACCEPTED ||
    static_cast<bool>(goal_handle_);

  canceled_goal_id_ = active_goal_id_;
  cancel_requested_ = true;
  pending_cancel_reason_ = reason ? reason : "";
  if (inflight) {
    navigation_transition::markCancelPending(active_navigation_token_);
  }

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
  if (!inflight) {
    cancel_requested_ = false;
  }
  goal_handle_.reset();
  state_ = InternalState::IDLE;
}

void ReliableNavigateToPose::beginCancelWait_(uint64_t goal_id, bool success_after)
{
  awaiting_cancel_confirm_ = true;
  success_pending_after_cancel_ = success_after;
  wait_cancel_goal_id_ = goal_id;
  cancel_wait_start_ = node_->now();
}

}  // namespace sentry_nav_bt

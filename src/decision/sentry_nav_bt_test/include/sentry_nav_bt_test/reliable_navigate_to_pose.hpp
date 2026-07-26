#ifndef SENTRY_NAV_BT_TEST_RELIABLE_NAVIGATE_TO_POSE_HPP_
#define SENTRY_NAV_BT_TEST_RELIABLE_NAVIGATE_TO_POSE_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace sentry_nav_bt_test
{

class ReliableNavigateToPose : public BT::StatefulActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  ReliableNavigateToPose(
    const std::string &name,
    const BT::NodeConfig &config,
    const rclcpp::Node::SharedPtr &node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class InternalState
  {
    IDLE,
    SENDING,
    ACCEPTED
  };

  enum class GoalStatus
  {
    NOT_REACHED,
    REACHED,
    REACHED_GUARD_UNSATISFIED
  };

  void resetRuntimeState_();
  bool refreshGoalInput_(bool *goal_changed = nullptr);
  bool isSuccessConditionSatisfied_(double *current_value = nullptr) const;
  GoalStatus evaluateGoalStatus_(double *distance = nullptr);
  void sendGoal_();
  void cancelGoal_(const char *reason);
  void beginCancelWait_(uint64_t goal_id, bool success_after);
  bool tryCancelGoal_(
    const rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr &handle,
    const char *reason);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;

  geometry_msgs::msg::PoseStamped current_goal_;
  std::string goal_name_;
  double reach_threshold_{0.30};
  double resend_interval_{0.50};
  double response_timeout_{1.00};
  double result_retry_delay_{0.50};
  double cancel_confirm_timeout_{1.50};
  int log_throttle_ms_{0};
  std::string success_condition_key_;
  std::string success_condition_comparison_{"eq"};
  double success_condition_threshold_{1.0};

  InternalState state_{InternalState::IDLE};
  rclcpp::Time last_send_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_log_time_{0, 0, RCL_ROS_TIME};
  bool result_ready_{false};
  bool result_success_{false};
  bool cancel_requested_{false};
  bool response_timeout_reported_{false};
  // 取消确认等待：取消旧目标后阻塞节点直到 Nav2 端确认终止，
  // 防止下一目标与取消请求在 bt_navigator 处交叠（pending goal 会屏蔽取消 → 跳点）
  bool awaiting_cancel_confirm_{false};
  bool success_pending_after_cancel_{false};
  uint64_t wait_cancel_goal_id_{0};
  rclcpp::Time cancel_wait_start_{0, 0, RCL_ROS_TIME};
  uint64_t active_goal_id_{0};
  uint64_t active_send_id_{0};
  uint64_t active_navigation_token_{0};
  uint64_t canceled_goal_id_{0};
  uint64_t seq_{0};
  uint64_t send_seq_{0};
  int send_attempts_{0};
  int retry_count_{0};
  mutable bool last_pose_invalid_logged_{false};
  std::string pending_cancel_reason_;
  rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr goal_handle_;
};

}  // namespace sentry_nav_bt_test

#endif  // SENTRY_NAV_BT_TEST_RELIABLE_NAVIGATE_TO_POSE_HPP_

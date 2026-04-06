#ifndef SENTRY_NAV_BT_TEST_CENTER_HOLD_VW_CONTROLLER_HPP_
#define SENTRY_NAV_BT_TEST_CENTER_HOLD_VW_CONTROLLER_HPP_

#include <cmath>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/blackboard.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sentry_nav_bt_test/referee_actions.hpp"

namespace sentry_nav_bt_test
{

class CenterHoldVwController
{
public:
  CenterHoldVwController(
    const rclcpp::Node::SharedPtr &node,
    const BT::Blackboard::Ptr &blackboard)
  : node_(node), blackboard_(blackboard)
  {
  }

  void start()
  {
    if (!node_ || !blackboard_) {
      throw std::runtime_error("CenterHoldVwController: node or blackboard is null");
    }

    control_publishers_ = std::make_shared<ControlTopicPublishers>(node_);
    vw_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(50),
      [this]() { updateCenterHoldVwCommand(); });
  }

private:
  double getBlackboardDouble(const std::string &key, double default_value) const
  {
    double value = default_value;
    blackboard_->get(key, value);
    return value;
  }

  bool isNearWaypoint(const std::string &waypoint_name, double threshold) const
  {
    bool current_pose_valid = false;
    if (!blackboard_->get("waypoint_now_valid", current_pose_valid) || !current_pose_valid) {
      return false;
    }

    geometry_msgs::msg::PoseStamped current_pose;
    geometry_msgs::msg::PoseStamped target_pose;

    if (!blackboard_->get("waypoint_now", current_pose)) {
      return false;
    }

    const std::string waypoint_key = "waypoint_" + waypoint_name;
    if (!blackboard_->get(waypoint_key, target_pose)) {
      return false;
    }

    const double dx = target_pose.pose.position.x - current_pose.pose.position.x;
    const double dy = target_pose.pose.position.y - current_pose.pose.position.y;
    return std::hypot(dx, dy) <= threshold;
  }

  bool isNearCurrentCenterGoal(bool currently_active) const
  {
    const double threshold = currently_active ?
      getBlackboardDouble("ul_center_hold_exit_distance_threshold", 0.55) :
      getBlackboardDouble("ul_center_hold_distance_threshold", 0.50);

    std::string goal_name = "center_point";
    blackboard_->get("ul_center_goal_name", goal_name);
    return isNearWaypoint(goal_name, threshold);
  }

  bool isCenterHoldActive(bool currently_active) const
  {
    int center_ready = 0;
    int retreat_active = 0;
    int ul_initialized = 0;
    uint16_t current_hp = 0;
    uint8_t game_progress = 0;

    const bool center_ready_ok =
      blackboard_->get("ul_center_ready", center_ready) && center_ready == 1;
    const bool retreat_inactive =
      blackboard_->get("ul_retreat_active", retreat_active) && retreat_active == 0;
    const bool initialized =
      blackboard_->get("ul_initialized", ul_initialized) && ul_initialized == 1;
    const bool hp_ok =
      blackboard_->get("current_hp", current_hp) && current_hp >= 150U;
    const bool match_started =
      blackboard_->get("game_progress", game_progress) && game_progress > 3U;
    const bool current_center_goal_nearby = isNearCurrentCenterGoal(currently_active);

    return match_started && initialized && retreat_inactive && hp_ok &&
      center_ready_ok && current_center_goal_nearby;
  }

  bool isNearCurrentUcFortressGoal(bool currently_active) const
  {
    const double threshold = currently_active ?
      getBlackboardDouble("uc_fortress_hold_exit_distance_threshold", 0.30) :
      getBlackboardDouble("uc_fortress_hold_distance_threshold", 0.25);

    std::string goal_name = "fortress";
    blackboard_->get("uc_fortress_goal_name", goal_name);
    return isNearWaypoint(goal_name, threshold);
  }

  bool isUcFortressHoldActive(bool currently_active) const
  {
    int fortress_hold_active = 0;
    uint16_t current_hp = 0;
    uint8_t game_progress = 0;

    const bool fortress_hold_enabled =
      blackboard_->get("uc_fortress_hold_active", fortress_hold_active) &&
      fortress_hold_active == 1;
    const bool hp_ok =
      blackboard_->get("current_hp", current_hp) && current_hp >= 150U;
    const bool match_started =
      blackboard_->get("game_progress", game_progress) && game_progress > 3U;
    const bool current_fortress_goal_nearby = isNearCurrentUcFortressGoal(currently_active);

    return match_started && hp_ok && fortress_hold_enabled && current_fortress_goal_nearby;
  }

  bool applyFortressDeactivateDebounce(bool raw_active, bool currently_active)
  {
    if (raw_active) {
      fortress_deactivate_false_ticks_ = 0;
      return true;
    }

    if (!currently_active) {
      fortress_deactivate_false_ticks_ = 0;
      return false;
    }

    ++fortress_deactivate_false_ticks_;
    if (fortress_deactivate_false_ticks_ < kFortressDeactivateDebounceTicks) {
      return true;
    }

    fortress_deactivate_false_ticks_ = 0;
    return false;
  }

  bool isAllyFortressRfidDetected() const
  {
    int ally_fortress_rfid_detected = 0;
    return blackboard_->get("rfid_ally_fortress_detected", ally_fortress_rfid_detected) &&
      ally_fortress_rfid_detected == 1;
  }

  void publishVwCommand(float value)
  {
    if (!control_publishers_) {
      return;
    }
    control_publishers_->publishVw(value);
  }

  void updateCenterHoldVwCommand()
  {
    const bool last_hold_vw_active =
      last_center_hold_vw_active_ || last_fortress_hold_vw_active_ ||
      last_rfid_fortress_vw_active_;
    const bool center_hold_active = isCenterHoldActive(last_center_hold_vw_active_);
    const bool fortress_hold_raw_active = isUcFortressHoldActive(last_fortress_hold_vw_active_);
    const bool fortress_hold_active =
      applyFortressDeactivateDebounce(fortress_hold_raw_active, last_fortress_hold_vw_active_);
    const bool rfid_fortress_active = isAllyFortressRfidDetected();
    const bool hold_vw_active = center_hold_active || fortress_hold_active || rfid_fortress_active;

    if (!vw_command_initialized_) {
      publishVwCommand(hold_vw_active ? 1.0f : 0.0f);
      vw_command_initialized_ = true;
      last_center_hold_vw_active_ = center_hold_active;
      last_fortress_hold_vw_active_ = fortress_hold_active;
      last_rfid_fortress_vw_active_ = rfid_fortress_active;
      return;
    }

    if (center_hold_active) {
      if (!last_center_hold_vw_active_) {
        RCLCPP_INFO(node_->get_logger(), "[UL] 中心驻守激活，开始持续发布 /vw = 1");
      }
    } else if (last_center_hold_vw_active_) {
      RCLCPP_INFO(node_->get_logger(), "[UL] 退出中心驻守，停止持续发布 /vw");
    }

    if (fortress_hold_active) {
      if (!last_fortress_hold_vw_active_) {
        RCLCPP_INFO(node_->get_logger(), "[UC] 堡垒驻守激活，开始持续发布 /vw = 1");
      }
    } else if (last_fortress_hold_vw_active_) {
      RCLCPP_INFO(node_->get_logger(), "[UC] 退出堡垒驻守，停止持续发布 /vw");
    }

    if (rfid_fortress_active) {
      if (!last_rfid_fortress_vw_active_) {
        RCLCPP_INFO(node_->get_logger(), "[RFID] 检测到己方堡垒增益点(bit17)，开始持续发布 /vw = 1");
      }
    } else if (last_rfid_fortress_vw_active_) {
      RCLCPP_INFO(node_->get_logger(), "[RFID] 己方堡垒增益点(bit17)丢失，停止 RFID 触发的 /vw 持续发布");
    }

    if (!last_hold_vw_active && hold_vw_active) {
      RCLCPP_INFO(node_->get_logger(), "[VW] 触发持续发布 /vw = 1");
    } else if (last_hold_vw_active && !hold_vw_active) {
      RCLCPP_INFO(node_->get_logger(), "[VW] 所有触发条件失效，停止持续发布 /vw = 1");
    }

    if (hold_vw_active) {
      publishVwCommand(1.0f);
    }

    // if (last_hold_vw_active && !hold_vw_active) {
    //   publishVwCommand(0.0f);
    // }

    last_center_hold_vw_active_ = center_hold_active;
    last_fortress_hold_vw_active_ = fortress_hold_active;
    last_rfid_fortress_vw_active_ = rfid_fortress_active;
  }

  rclcpp::Node::SharedPtr node_;
  BT::Blackboard::Ptr blackboard_;
  std::shared_ptr<ControlTopicPublishers> control_publishers_;
  rclcpp::TimerBase::SharedPtr vw_timer_;
  bool last_center_hold_vw_active_{false};
  bool last_fortress_hold_vw_active_{false};
  bool last_rfid_fortress_vw_active_{false};
  bool vw_command_initialized_{false};
  int fortress_deactivate_false_ticks_{0};

  static constexpr int kFortressDeactivateDebounceTicks = 3;
};

}  // namespace sentry_nav_bt_test

#endif  // SENTRY_NAV_BT_TEST_CENTER_HOLD_VW_CONTROLLER_HPP_

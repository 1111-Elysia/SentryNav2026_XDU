#ifndef SENTRY_NAV_BT_TEST_RUNTIME_CONFIG_HPP_
#define SENTRY_NAV_BT_TEST_RUNTIME_CONFIG_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/blackboard.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"

namespace sentry_nav_bt_test
{

class RuntimeConfigManager
{
public:
  RuntimeConfigManager(
    const rclcpp::Node::SharedPtr & node,
    const BT::Blackboard::Ptr & blackboard);

  bool applyCurrentParameters();

private:
  struct Config
  {
    std::string goal_name{"init"};
    bool use_custom_pose{false};
    double goal_x{0.0};
    double goal_y{0.0};
    double goal_yaw{0.0};
    int move_posture{3};
    int wait_posture{1};
    double reach_threshold{0.25};
    double wait_time_threshold{5.0};
  };

  Config currentConfig() const;
  bool validate(const Config & config, std::string & reason) const;
  void applyToBlackboard(const Config & config);
  rcl_interfaces::msg::SetParametersResult onParametersChanged(
    const std::vector<rclcpp::Parameter> & parameters);

  rclcpp::Node::SharedPtr node_;
  BT::Blackboard::Ptr blackboard_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
};

}  // namespace sentry_nav_bt_test

#endif  // SENTRY_NAV_BT_TEST_RUNTIME_CONFIG_HPP_

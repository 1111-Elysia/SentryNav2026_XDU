#ifndef SENTRY_NAV_BT_TEST_CHECK_CONDITION_HPP_
#define SENTRY_NAV_BT_TEST_CHECK_CONDITION_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "sentry_nav_bt_test/blackboard_utils.hpp"

namespace sentry_nav_bt_test
{

  class CheckCondition : public BT::ConditionNode
  {
  public:
    CheckCondition(
        const std::string &name,
        const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<std::string>("key_name", "黑板中要检查的键名"),
          BT::InputPort<std::string>("comparison", "比较操作符：'gt'(大于), 'lt'(小于), 'eq'(等于), 'gte'(大于等于), 'lte'(小于等于), 'neq'(不等于)"),
          BT::InputPort<double>("threshold", "用于比较的阈值")};
    }

    BT::NodeStatus tick() override
    {
      // 获取参数
      std::string key_name;
      std::string comparison;
      double threshold;

      if (!getInput("key_name", key_name))
      {
        RCLCPP_ERROR(
            rclcpp::get_logger("CheckCondition"),
            "缺少必要参数 'key_name'");
        return BT::NodeStatus::FAILURE;
      }

      if (!getInput("comparison", comparison))
      {
        RCLCPP_ERROR(
            rclcpp::get_logger("CheckCondition"),
            "缺少必要参数 'comparison'");
        return BT::NodeStatus::FAILURE;
      }

      if (!getInput("threshold", threshold))
      {
        RCLCPP_ERROR(
            rclcpp::get_logger("CheckCondition"),
            "缺少必要参数 'threshold'");
        return BT::NodeStatus::FAILURE;
      }

      // 从黑板获取键值
      auto blackboard = this->config().blackboard;
      double value = 0.0;

      // 使用工具函数获取值
      if (!blackboard_utils::getValue(blackboard, key_name, value, "CheckCondition"))
      {
        RCLCPP_WARN(
            rclcpp::get_logger("CheckCondition"),
            "无法从黑板获取键 '%s'", key_name.c_str());
        return BT::NodeStatus::FAILURE;
      }

      // 使用工具函数执行比较
      bool comparison_result = blackboard_utils::compareValues(
          value, comparison, threshold, "CheckCondition");

      // 日志记录
      RCLCPP_DEBUG(
          rclcpp::get_logger("CheckCondition"),
          "黑板键 '%s' 的值: %f %s 阈值: %f => %s",
          key_name.c_str(), value, comparison.c_str(), threshold,
          comparison_result ? "SUCCESS" : "FAILURE");

      // 返回比较结果
      return comparison_result ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
  };

} // namespace sentry_nav_bt_test

#endif // SENTRY_NAV_BT_TEST_CHECK_CONDITION_HPP_
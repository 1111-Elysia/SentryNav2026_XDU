#ifndef SENTRY_NAV_BT_TEST_COMPARE_VALUES_HPP_
#define SENTRY_NAV_BT_TEST_COMPARE_VALUES_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "sentry_nav_bt_test/blackboard_utils.hpp"

namespace sentry_nav_bt_test
{

  class CompareValues : public BT::ConditionNode
  {
  public:
    CompareValues(
        const std::string &name,
        const BT::NodeConfig &config)
        : BT::ConditionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<std::string>("first_key", "黑板中的第一个键名"),
          BT::InputPort<std::string>("second_key", "黑板中的第二个键名"),
          BT::InputPort<std::string>("comparison", "比较操作符：'gt'(大于), 'lt'(小于), 'eq'(等于), 'gte'(大于等于), 'lte'(小于等于), 'neq'(不等于)")};
    }

    BT::NodeStatus tick() override
    {
      // 获取参数
      std::string first_key;
      std::string second_key;
      std::string comparison;

      if (!getInput("first_key", first_key))
      {
        RCLCPP_ERROR(
            rclcpp::get_logger("CompareValues"),
            "缺少必要参数 'first_key'");
        return BT::NodeStatus::FAILURE;
      }

      if (!getInput("second_key", second_key))
      {
        RCLCPP_ERROR(
            rclcpp::get_logger("CompareValues"),
            "缺少必要参数 'second_key'");
        return BT::NodeStatus::FAILURE;
      }

      if (!getInput("comparison", comparison))
      {
        RCLCPP_ERROR(
            rclcpp::get_logger("CompareValues"),
            "缺少必要参数 'comparison'");
        return BT::NodeStatus::FAILURE;
      }

      // 从黑板获取两个键值
      auto blackboard = this->config().blackboard;
      double first_value = 0.0;
      double second_value = 0.0;

      // 使用工具函数获取第一个值
      if (!blackboard_utils::getValue(blackboard, first_key, first_value, "CompareValues"))
      {
        RCLCPP_WARN(
            rclcpp::get_logger("CompareValues"),
            "无法从黑板获取键 '%s'", first_key.c_str());
        return BT::NodeStatus::FAILURE;
      }

      // 使用工具函数获取第二个值
      if (!blackboard_utils::getValue(blackboard, second_key, second_value, "CompareValues"))
      {
        RCLCPP_WARN(
            rclcpp::get_logger("CompareValues"),
            "无法从黑板获取键 '%s'", second_key.c_str());
        return BT::NodeStatus::FAILURE;
      }

      // 使用工具函数执行比较操作
      bool comparison_result = blackboard_utils::compareValues(
          first_value, comparison, second_value, "CompareValues");

      // 日志记录
      RCLCPP_DEBUG(
          rclcpp::get_logger("CompareValues"),
          "比较结果: %s(%f) %s %s(%f) => %s",
          first_key.c_str(), first_value, comparison.c_str(),
          second_key.c_str(), second_value,
          comparison_result ? "SUCCESS" : "FAILURE");

      // 返回比较结果
      return comparison_result ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
  };

} // namespace sentry_nav_bt_test

#endif // SENTRY_NAV_BT_TEST_COMPARE_VALUES_HPP_
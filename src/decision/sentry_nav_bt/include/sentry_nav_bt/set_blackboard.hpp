#ifndef SENTRY_NAV_BT_SET_BLACKBOARD_VALUE_HPP_
#define SENTRY_NAV_BT_SET_BLACKBOARD_VALUE_HPP_

#include <string>
#include "behaviortree_cpp/action_node.h"
#include "sentry_nav_bt/blackboard_utils.hpp"
#include "rclcpp/rclcpp.hpp"

namespace sentry_nav_bt
{
class SetBlackboardValue : public BT::SyncActionNode
{
public:
  SetBlackboardValue(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("key", "要设置的黑板键名"),
      BT::InputPort<std::string>("value", "要设置的值"),
      BT::InputPort<std::string>("type", "值的类型 (int, double, bool, string, uint8)", "auto")
    };
  }

  BT::NodeStatus tick() override
  {
    auto blackboard = config().blackboard;
    if (!blackboard) {
      RCLCPP_ERROR(rclcpp::get_logger("SetBlackboardValue"), "无法获取黑板");
      return BT::NodeStatus::FAILURE;
    }

    std::string key;
    if (!getInput("key", key)) {
      RCLCPP_ERROR(rclcpp::get_logger("SetBlackboardValue"), "未指定键名");
      return BT::NodeStatus::FAILURE;
    }

    std::string value;
    if (!getInput("value", value)) {
      RCLCPP_ERROR(rclcpp::get_logger("SetBlackboardValue"), "未指定值");
      return BT::NodeStatus::FAILURE;
    }

    std::string type;
    getInput("type", type); // 默认为 "auto"

    auto logger = rclcpp::get_logger("SetBlackboardValue");
    
    // 检查键是否存在，如果存在冲突则先移除 (可选)
    // if (blackboard->exists(key) && type == "auto") {
    //   // 可以选择先移除有冲突的键，但这可能会影响其他依赖于此键的节点
    //   // blackboard->remove(key); 
    // }

    // 根据指定类型进行转换并存储
    if (type == "int" || type == "integer") {
      try {
        int value_int = std::stoi(value);
        blackboard->set(key, value_int);
        RCLCPP_DEBUG(logger, "设置黑板值 %s = %d (整数型)", key.c_str(), value_int);
        return BT::NodeStatus::SUCCESS;
      } catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "将 '%s' 转换为整数时出错: %s", value.c_str(), e.what());
        return BT::NodeStatus::FAILURE;
      }
    } 
    else if (type == "double" || type == "float") {
      try {
        double value_double = std::stod(value);
        blackboard->set(key, value_double);
        RCLCPP_DEBUG(logger, "设置黑板值 %s = %f (浮点型)", key.c_str(), value_double);
        return BT::NodeStatus::SUCCESS;
      } catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "将 '%s' 转换为浮点数时出错: %s", value.c_str(), e.what());
        return BT::NodeStatus::FAILURE;
      }
    }
    else if (type == "bool" || type == "boolean") {
      bool value_bool = (value == "true" || value == "1" || value == "True");
      blackboard->set(key, value_bool);
      RCLCPP_DEBUG(logger, "设置黑板值 %s = %s (布尔型)", key.c_str(), value_bool ? "true" : "false");
      return BT::NodeStatus::SUCCESS;
    }
    else if (type == "uint8" || type == "unsigned char" || type == "byte") {
      try {
        uint8_t value_uint8 = static_cast<uint8_t>(std::stoi(value));
        blackboard->set(key, value_uint8);
        RCLCPP_DEBUG(logger, "设置黑板值 %s = %d (无符号字符型)", key.c_str(), static_cast<int>(value_uint8));
        return BT::NodeStatus::SUCCESS;
      } catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "将 '%s' 转换为无符号字符型时出错: %s", value.c_str(), e.what());
        return BT::NodeStatus::FAILURE;
      }
    }
    else if (type == "uint16" || type == "unsigned short") {
      try {
        uint16_t value_uint16 = static_cast<uint16_t>(std::stoi(value));
        blackboard->set(key, value_uint16);
        RCLCPP_DEBUG(logger, "设置黑板值 %s = %d (无符号短整型)", key.c_str(), value_uint16);
        return BT::NodeStatus::SUCCESS;
      } catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "将 '%s' 转换为无符号短整型时出错: %s", value.c_str(), e.what());
        return BT::NodeStatus::FAILURE;
      }
    }
    else if (type == "string" || type == "std::string") {
      blackboard->set(key, value);
      RCLCPP_DEBUG(logger, "设置黑板值 %s = %s (字符串型)", key.c_str(), value.c_str());
      return BT::NodeStatus::SUCCESS;
    }
    else if (type == "auto") {
      // 使用原有的自动检测类型逻辑
      if (blackboard_utils::setValue(blackboard, key, value, logger)) {
        return BT::NodeStatus::SUCCESS;
      } else {
        return BT::NodeStatus::FAILURE;
      }
    }
    else {
      RCLCPP_ERROR(logger, "不支持的类型: %s", type.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }
};
}  // namespace sentry_nav_bt

#endif  // SENTRY_NAV_BT_SET_BLACKBOARD_VALUE_HPP_
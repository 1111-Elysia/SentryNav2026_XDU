#ifndef SENTRY_NAV_BT_TEST_PRINT_BLACKBOARD_NODE_HPP_
#define SENTRY_NAV_BT_TEST_PRINT_BLACKBOARD_NODE_HPP_

#include <string>
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sentry_nav_bt_test/blackboard_utils.hpp"

namespace sentry_nav_bt_test
{

/**
 * @brief 打印黑板值的简单行为节点
 */
class PrintBlackboardValue : public BT::ActionNodeBase
{
public:
  /**
   * @brief 构造函数
   * @param name 节点名称
   * @param config 节点配置
   */
  PrintBlackboardValue(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ActionNodeBase(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  }

  /**
   * @brief 定义节点的输入输出端口
   * @return 端口列表
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("key", "要打印的黑板键名"),
      BT::InputPort<std::string>("prefix", "前缀文本", "黑板值")
    };
  }

  /**
   * @brief 执行节点的主要逻辑
   * @return 节点状态
   */
  BT::NodeStatus tick() override
  {
    std::string key;
    std::string prefix;
    
    if (!getInput("key", key)) {
      RCLCPP_ERROR(node_->get_logger(), "未提供要打印的黑板键名");
      return BT::NodeStatus::FAILURE;
    }
    
    getInput("prefix", prefix);
    
    auto blackboard = this->config().blackboard;
    
    // 使用工具函数打印值
    if (!blackboard_utils::printValue(blackboard, key, prefix, node_->get_logger())) {
      // 如果所有类型都失败，打印警告信息
      RCLCPP_WARN(node_->get_logger(), "%s: 无法打印键'%s'的值(类型不支持或键不存在)", 
               prefix.c_str(), key.c_str());
    }
    
    return BT::NodeStatus::SUCCESS;
  }

  /**
   * @brief 处理节点被中断或停止时的逻辑
   */
  void halt() override 
  {
    RCLCPP_DEBUG(node_->get_logger(), "PrintBlackboardValue节点被中断");
  }

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace sentry_nav_bt_test

#endif  // SENTRY_NAV_BT_TEST_PRINT_BLACKBOARD_NODE_HPP_
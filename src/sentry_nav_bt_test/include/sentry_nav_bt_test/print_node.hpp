#ifndef SENTRY_NAV_BT_TEST_PRINT_NODE_HPP_
#define SENTRY_NAV_BT_TEST_PRINT_NODE_HPP_

#include <fstream>
#include <mutex>
#include <string>
#include <memory>
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace sentry_nav_bt_test
{

/**
 * @brief 简单打印节点，用于打印给定的输入内容
 */
class PrintNode : public BT::SyncActionNode
{
public:
  /**
   * @brief 构造函数
   * @param name 节点名称
   * @param config 节点配置
   */
  PrintNode(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  }

  /**
   * @brief 定义节点的输入端口
   * @return 端口列表
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("message", "要打印的消息")
    };
  }

  /**
   * @brief 执行节点的主要逻辑
   * @return 节点状态
   */
  BT::NodeStatus tick() override
  {
    std::string message;
    
    if (!getInput("message", message)) {
      RCLCPP_ERROR(node_->get_logger(), "未提供要打印的消息");
      return BT::NodeStatus::FAILURE;
    }
    
    // 简单打印消息，使用INFO级别
    RCLCPP_INFO(node_->get_logger(), "%s", message.c_str());

    std::string log_file;
    if (config().blackboard &&
        config().blackboard->get("bt_message_log_file", log_file) &&
        !log_file.empty())
    {
      static std::mutex log_mutex;
      std::lock_guard<std::mutex> lock(log_mutex);

      std::ofstream ofs(log_file, std::ios::app);
      if (ofs.is_open()) {
        ofs << "[" << node_->now().seconds() << "] " << message << '\n';
      } else {
        RCLCPP_WARN_THROTTLE(
          node_->get_logger(),
          *node_->get_clock(),
          2000,
          "无法打开行为树消息日志文件: %s",
          log_file.c_str());
      }
    }
    
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace sentry_nav_bt_test

#endif  // SENTRY_NAV_BT_TEST_PRINT_NODE_HPP_

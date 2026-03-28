#ifndef SENTRY_NAV_BT_TEST_PRINT_BLACKBOARD_NODE_HPP_
#define SENTRY_NAV_BT_TEST_PRINT_BLACKBOARD_NODE_HPP_

#include <cstdint>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
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
    if (config.blackboard) {
      config.blackboard->get("bt_message_log_process_label", process_label_);
    }
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
    std::string formatted_value;
    
    if (!blackboard_utils::formatValue(blackboard, key, formatted_value)) {
      // 如果所有类型都失败，打印警告信息
      RCLCPP_WARN(node_->get_logger(), "%s: 无法打印键'%s'的值(类型不支持或键不存在)", 
               prefix.c_str(), key.c_str());
      return BT::NodeStatus::SUCCESS;
    }

    const auto log_message =
      blackboard_utils::formatKeyValueMessage(key, prefix, formatted_value);
    RCLCPP_INFO(node_->get_logger(), "%s", log_message.c_str());

    std::string log_file;
    std::string history_log_file;
    if (config().blackboard &&
        config().blackboard->get("bt_message_log_file", log_file))
    {
      static std::mutex log_mutex;
      std::lock_guard<std::mutex> lock(log_mutex);

      if (!log_file.empty()) {
        writeLogLine(log_file, log_message);
      }

      if (config().blackboard->get("bt_message_log_history_file", history_log_file) &&
          !history_log_file.empty() &&
          history_log_file != log_file)
      {
        writeLogLine(history_log_file, log_message);
      }
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
  std::string formatLogLine(const std::string &message) const
  {
    const auto now = node_->get_clock()->now();
    const std::int64_t total_nanoseconds = now.nanoseconds();
    const std::int64_t seconds = total_nanoseconds / 1000000000LL;
    const std::int64_t nanoseconds = total_nanoseconds % 1000000000LL;

    std::ostringstream oss;
    if (!process_label_.empty()) {
      oss << "[" << process_label_ << "] ";
    }
    oss << "[INFO] ["
        << seconds << "."
        << std::setw(9) << std::setfill('0') << nanoseconds
        << "] ["
        << node_->get_logger().get_name()
        << "]: "
        << message;
    return oss.str();
  }

  bool writeLogLine(const std::string &log_file, const std::string &message)
  {
    std::ofstream ofs(log_file, std::ios::app);
    if (!ofs.is_open()) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        2000,
        "无法打开行为树消息日志文件: %s",
        log_file.c_str());
      return false;
    }

    ofs << formatLogLine(message) << '\n';
    return true;
  }

  rclcpp::Node::SharedPtr node_;
  std::string process_label_;
};

}  // namespace sentry_nav_bt_test

#endif  // SENTRY_NAV_BT_TEST_PRINT_BLACKBOARD_NODE_HPP_

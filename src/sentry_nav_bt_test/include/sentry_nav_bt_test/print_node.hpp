#ifndef SENTRY_NAV_BT_TEST_PRINT_NODE_HPP_
#define SENTRY_NAV_BT_TEST_PRINT_NODE_HPP_

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <cstdint>
#include <mutex>
#include <memory>
#include <sstream>
#include <string>
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
    if (config.blackboard) {
      config.blackboard->get("bt_message_log_process_label", process_label_);
    }
  }

  /**
   * @brief 定义节点的输入端口
   * @return 端口列表
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("message", "要打印的消息"),
      BT::InputPort<int>("throttle_ms", 0, "同一节点重复打印的最小间隔(ms)，0表示不节流")
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

    int throttle_ms = 0;
    getInput("throttle_ms", throttle_ms);
    throttle_ms = std::max(throttle_ms, 0);

    const auto now_tp = std::chrono::steady_clock::now();
    const bool message_changed = !has_last_logged_message_ || message != last_logged_message_;
    if (!message_changed && throttle_ms > 0 && has_last_log_time_) {
      const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now_tp - last_log_time_).count();
      if (elapsed_ms < throttle_ms) {
        return BT::NodeStatus::SUCCESS;
      }
    }

    // 简单打印消息，使用INFO级别
    RCLCPP_INFO(node_->get_logger(), "%s", message.c_str());
    last_logged_message_ = message;
    last_log_time_ = now_tp;
    has_last_logged_message_ = true;
    has_last_log_time_ = true;

    std::string log_file;
    std::string history_log_file;
    if (config().blackboard &&
        config().blackboard->get("bt_message_log_file", log_file))
    {
      static std::mutex log_mutex;
      std::lock_guard<std::mutex> lock(log_mutex);

      if (!log_file.empty()) {
        writeLogLine(log_file, message);
      }

      if (config().blackboard->get("bt_message_log_history_file", history_log_file) &&
          !history_log_file.empty() &&
          history_log_file != log_file)
      {
        writeLogLine(history_log_file, message);
      }
    }
    
    return BT::NodeStatus::SUCCESS;
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
  std::string last_logged_message_;
  std::chrono::steady_clock::time_point last_log_time_{};
  bool has_last_logged_message_{false};
  bool has_last_log_time_{false};
};

}  // namespace sentry_nav_bt_test

#endif  // SENTRY_NAV_BT_TEST_PRINT_NODE_HPP_

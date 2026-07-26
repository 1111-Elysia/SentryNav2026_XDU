#ifndef SENTRY_NAV_BT_TOPIC_LISTENER_HPP_
#define SENTRY_NAV_BT_TOPIC_LISTENER_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/blackboard.h"
#include "rclcpp/rclcpp.hpp"

namespace sentry_nav_bt
{

/**
 * @brief 负责将裁判系统、TF 和比赛状态同步到行为树黑板。
 *
 * 具体订阅与状态实现隐藏在 .cpp 中，避免把所有消息类型和 TF 依赖
 * 传播给使用该管理器的编译单元。
 */
class BlackboardManager
{
public:
    BlackboardManager(
        rclcpp::Node::SharedPtr node,
        BT::Blackboard::Ptr blackboard);
    ~BlackboardManager();

    BlackboardManager(const BlackboardManager &) = delete;
    BlackboardManager &operator=(const BlackboardManager &) = delete;
    BlackboardManager(BlackboardManager &&) noexcept;
    BlackboardManager &operator=(BlackboardManager &&) noexcept;

    void bb_manager_init();
    void load_waypoints(const std::string &json_file_path);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace sentry_nav_bt

#endif // SENTRY_NAV_BT_TOPIC_LISTENER_HPP_

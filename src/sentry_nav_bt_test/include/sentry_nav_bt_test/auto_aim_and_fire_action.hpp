#ifndef AUTO_AIM_AND_FIRE_ACTION_HPP
#define AUTO_AIM_AND_FIRE_ACTION_HPP

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace sentry_nav_bt_test {

class AutoAimAndFire : public BT::SyncActionNode {
public:
    AutoAimAndFire(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("target_type") };
    }

    BT::NodeStatus tick() override {
        // 1. 获取目标类型
        std::string target;
        if (!getInput("target_type", target)) {
            return BT::NodeStatus::FAILURE;
        }

        // 2. 模拟开火过程 (打印日志)
        // 在实际开发中，这里会调用自瞄算法或发布ROS话题
        std::cout << "\033[1;31m"; // 红色字体
        std::cout << ">>> [MOCK] 正在瞄准目标: " << target << " ... 锁定! 开火!" << std::endl;
        std::cout << "\033[0m";    // 重置字体

        // 3. 假装成功
        return BT::NodeStatus::SUCCESS;
    }
};

} // namespace sentry_nav_bt_test
#endif
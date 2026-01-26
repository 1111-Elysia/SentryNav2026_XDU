#ifndef SENTRY_NAV_BT_TEST_REFEREE_ACTIONS_HPP_
#define SENTRY_NAV_BT_TEST_REFEREE_ACTIONS_HPP_

#include <string>
#include <memory>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rm_referee_msgs/srv/tx.hpp"
#include "sentry_nav_bt_test/cmd_utils.hpp" 

namespace sentry_nav_bt_test
{
    // ==========================================
    // 基类：RefereeActionBase
    // ==========================================
    class RefereeActionBase : public BT::SyncActionNode
    {
    public:
        RefereeActionBase(const std::string &name, const BT::NodeConfiguration &config);

    protected:
        // 尝试初始化工具类 (如果 Robot ID 还没拿到，返回 false)
        bool initUtils();

        // 异步发送数据包
        void send_packet(const std::vector<uint8_t> &data);

        rclcpp::Node::SharedPtr node_;
        rclcpp::Client<rm_referee_msgs::srv::Tx>::SharedPtr client_;
        std::shared_ptr<SentryRefereeUtils> utils_;
    };

    // ==========================================
    // 动作 1: SetSentryPosture (切换姿态 - 闭环控制)
    // XML: <Action ID="SetSentryPosture" mode="1" timeout_ms="1000"/>
    // ==========================================
    class SetSentryPosture : public RefereeActionBase
    {
    public:
        SetSentryPosture(const std::string &name, const BT::NodeConfiguration &config);

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;
    };

    // ==========================================
    // 动作 2: RequestActivateRune (请求开符 - 鲁棒发送)
    // XML: <Action ID="RequestActivateRune" posture="0"/>
    // ==========================================
    class RequestActivateRune : public RefereeActionBase
    {
    public:
        RequestActivateRune(const std::string &name, const BT::NodeConfiguration &config);

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;
    };

} // namespace sentry_nav_bt_test

#endif // SENTRY_NAV_BT_TEST_REFEREE_ACTIONS_HPP_
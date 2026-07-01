#ifndef SENTRY_NAV_BT_TEST_REFEREE_ACTIONS_HPP_
#define SENTRY_NAV_BT_TEST_REFEREE_ACTIONS_HPP_

#include <chrono>
#include <string>
#include <memory>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rm_referee_msgs/srv/tx.hpp"
#include "sentry_nav_bt_test/cmd_utils.hpp" 
#include "sentry_nav_bt_test/referee/control_topic_publishers.hpp"

namespace sentry_nav_bt_test
{
    // 裁判系统动作基类：负责拿 robot_id、创建 /rm_referee/tx client、同步发送交互包。
    class RefereeActionBase : public BT::SyncActionNode
    {
    public:
        RefereeActionBase(const std::string &name, const BT::NodeConfig &config);

    protected:
        // 尝试初始化工具类 (如果 Robot ID 还没拿到，返回 false)
        bool initUtils();

        // 同步发送数据包，并等待 /rm_referee/tx 返回结果
        bool send_packet(
            const std::vector<uint8_t> &data,
            std::chrono::milliseconds response_timeout = std::chrono::milliseconds(200));

        rclcpp::Node::SharedPtr node_;
        rclcpp::Client<rm_referee_msgs::srv::Tx>::SharedPtr client_;
        std::shared_ptr<SentryRefereeUtils> utils_;
    };

    // 动作 1：维持哨兵姿态
    // XML: <MaintainSentryPosture mode="3"/>
    class MaintainSentryPosture : public RefereeActionBase
    {
    public:
        MaintainSentryPosture(const std::string &name, const BT::NodeConfig &config);

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;

    private:
        int last_confirmed_mode_{-1};
    };

    // 将已耗尽时长的强化姿态回退为对应普通姿态。
    // XML: <ResolveSentryPosture requested_mode="4" resolved_mode="{effective_mode}"/>
    class ResolveSentryPosture : public BT::SyncActionNode
    {
    public:
        ResolveSentryPosture(const std::string &name, const BT::NodeConfig &config);

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;

    private:
        int last_requested_mode_{-1};
        int last_resolved_mode_{-1};
    };

    // 动作 2：确认免费复活
    // XML: <ConfirmResurrection posture="0" burst_count="3" burst_interval_ms="20"/>
    class ConfirmResurrection : public RefereeActionBase
    {
    public:
        ConfirmResurrection(const std::string &name, const BT::NodeConfig &config);

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;

    private:
        std::chrono::steady_clock::time_point last_send_time_{};
    };

    // 动作 3：补血点兑换 17mm 允许发弹量
    // XML: <BuySentryProjectile target_allowance="150" max_exchange_projectile="300"/>
    class BuySentryProjectile : public RefereeActionBase
    {
    public:
        BuySentryProjectile(const std::string &name, const BT::NodeConfig &config);

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;

    private:
        int resolveRequestedPosture(int requested_posture) const;
        void setBuyStatus(
            const std::string &result,
            int buy_amount,
            int exchange_target,
            bool tx_ok) const;

        std::chrono::steady_clock::time_point last_send_time_{};
    };

    // 动作 4：打能量机关
    // 顺序：scan_mode=true -> yaw_controller=0 -> autoshoot=true -> 等待 2s -> 发送激活请求
    // XML: <EngageRune rune_type="small" posture="1" timeout_ms="30000" request_interval_ms="1000"/>
    class EngageRune : public BT::StatefulActionNode
    {
    public:
        EngageRune(const std::string &name, const BT::NodeConfig &config);

        static BT::PortsList providedPorts();

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        enum class RuneType
        {
            SMALL,
            LARGE
        };

        bool initUtils();
        bool send_packet(
            const std::vector<uint8_t> &data,
            std::chrono::milliseconds response_timeout = std::chrono::milliseconds(200));
        bool publishScanMode(bool enabled);
        bool publishAutoShoot(bool enabled);
        bool triggerYawController();
        // 确保打符所需输出已经按顺序打开；已打开的输出不会重复发布。
        bool ensureEngageOutputs();
        void cleanupOutputs();
        bool tryGetCanActivateRune(int &can_activate) const;
        int resolveRequestedPosture() const;
        rm_protocol::SentryPosture resolvePostureEnum() const;
        const char *runeTypeName() const;
        void setRuneOutcome(bool success, const std::string &result) const;

        rclcpp::Node::SharedPtr node_;
        rclcpp::Client<rm_referee_msgs::srv::Tx>::SharedPtr client_;
        std::shared_ptr<ControlTopicPublishers> control_publishers_;
        std::shared_ptr<SentryRefereeUtils> utils_;

        RuneType rune_type_{RuneType::SMALL};
        int requested_posture_{1};
        int timeout_ms_{45000};
        int request_interval_ms_{1000};
        std::chrono::steady_clock::time_point start_time_{};
        std::chrono::steady_clock::time_point last_request_time_{};
        std::chrono::steady_clock::time_point yaw_controller_trigger_time_{};
        bool scan_mode_yaw_control_enabled_{false};
        bool yaw_controller_triggered_{false};
        bool auto_shoot_enabled_{false};
        bool saw_activating_state_{false};
        std::string active_rune_status_key_;
    };

    // 动作 5：打前哨站
    // 顺序：scan_mode=true -> yaw_controller=1 -> outpost_mode_type=true
    // XML: <EngageOutpost timeout_ms="70000"/>
    class EngageOutpost : public BT::StatefulActionNode
    {
    public:
        EngageOutpost(const std::string &name, const BT::NodeConfig &config);

        static BT::PortsList providedPorts();

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        bool initUtils();
        bool send_packet(
            const std::vector<uint8_t> &data,
            std::chrono::milliseconds response_timeout = std::chrono::milliseconds(200));
        bool publishScanMode(bool enabled);
        bool triggerYawController();
        bool publishOutpostMode(bool enabled);
        bool requestOutpostAttackPosture();
        // 确保打前哨站所需输出已经按顺序打开；已打开的输出不会重复发布。
        bool ensureEngageOutputs();
        bool isEnemyOutpostDestroyed() const;
        void cleanupOutputs();
        void setOutpostOutcome(bool success, const std::string &result) const;

        rclcpp::Node::SharedPtr node_;
        rclcpp::Client<rm_referee_msgs::srv::Tx>::SharedPtr client_;
        std::shared_ptr<ControlTopicPublishers> control_publishers_;
        std::shared_ptr<SentryRefereeUtils> utils_;

        int timeout_ms_{45000};
        std::chrono::steady_clock::time_point start_time_{};
        bool scan_mode_yaw_control_enabled_{false};
        bool yaw_controller_triggered_{false};
        bool outpost_mode_enabled_{false};
        bool outpost_attack_posture_requested_{false};
    };

} // namespace sentry_nav_bt_test

#endif // SENTRY_NAV_BT_TEST_REFEREE_ACTIONS_HPP_

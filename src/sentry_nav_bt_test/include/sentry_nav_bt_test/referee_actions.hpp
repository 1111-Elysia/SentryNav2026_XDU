#ifndef SENTRY_NAV_BT_TEST_REFEREE_ACTIONS_HPP_
#define SENTRY_NAV_BT_TEST_REFEREE_ACTIONS_HPP_

#include <chrono>
#include <string>
#include <memory>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rm_referee_msgs/srv/tx.hpp"
#include "sentry_msgs/msg/scan_mode.hpp"
#include "sentry_msgs/msg/vw.hpp"
#include "sentry_nav_bt_test/cmd_utils.hpp" 
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"

namespace sentry_nav_bt_test
{
    // 统一管理裁判/底盘控制相关 topic 发布器，供多个动作节点复用。
    class ControlTopicPublishers
    {
    public:
        explicit ControlTopicPublishers(const rclcpp::Node::SharedPtr &node)
            : node_(node)
        {
        }

        bool publishVw(float value)
        {
            ensureInitialized();
            if (!vw_pub_) {
                logMissingPublisher("/vw");
                return false;
            }

            sentry_msgs::msg::Vw msg;
            msg.vw = value;
            vw_pub_->publish(msg);
            return true;
        }

        bool publishScanMode(bool enabled)
        {
            ensureInitialized();
            if (!scan_mode_pub_) {
                logMissingPublisher("/scan_mod_type");
                return false;
            }

            sentry_msgs::msg::ScanMode msg;
            msg.scan_mod_type = enabled;
            scan_mode_pub_->publish(msg);
            return true;
        }

        bool publishAutoShoot(bool enabled)
        {
            ensureInitialized();
            if (!auto_shoot_pub_) {
                logMissingPublisher("/auto_shoot_type");
                return false;
            }

            std_msgs::msg::Bool msg;
            msg.data = enabled;
            auto_shoot_pub_->publish(msg);
            return true;
        }

        bool publishYawController(int target)
        {
            ensureInitialized();
            if (!yaw_controller_pub_) {
                logMissingPublisher("/yaw_controller");
                return false;
            }

            std_msgs::msg::Int32 msg;
            msg.data = target;
            yaw_controller_pub_->publish(msg);
            return true;
        }

        bool publishOutpostMode(bool enabled)
        {
            ensureInitialized();
            if (!outpost_mode_pub_) {
                logMissingPublisher("/outpost_mode_type");
                return false;
            }

            std_msgs::msg::Bool msg;
            msg.data = enabled;
            outpost_mode_pub_->publish(msg);
            return true;
        }

    private:
        void ensureInitialized()
        {
            if (initialized_ || !node_) {
                return;
            }

            vw_pub_ = node_->create_publisher<sentry_msgs::msg::Vw>("/vw", 10);
            scan_mode_pub_ = node_->create_publisher<sentry_msgs::msg::ScanMode>("/scan_mod_type", 10);
            auto_shoot_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/auto_shoot_type", 10);
            yaw_controller_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/yaw_controller", 10);
            outpost_mode_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/outpost_mode_type", 10);
            initialized_ = true;
        }

        void logMissingPublisher(const char *topic_name) const
        {
            if (node_) {
                RCLCPP_ERROR(node_->get_logger(), "未初始化 %s 发布器", topic_name);
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("ControlTopicPublishers"), "node 为空，无法发布 %s", topic_name);
            }
        }

        rclcpp::Node::SharedPtr node_;
        bool initialized_{false};
        rclcpp::Publisher<sentry_msgs::msg::Vw>::SharedPtr vw_pub_;
        rclcpp::Publisher<sentry_msgs::msg::ScanMode>::SharedPtr scan_mode_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr auto_shoot_pub_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr yaw_controller_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr outpost_mode_pub_;
    };

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

    // 动作 3：打能量机关
    // 顺序：scan_mode=false -> yaw_controller=0 -> autoshoot=true -> 发送激活请求
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
        bool tryGetRuneStatus(int &status) const;
        bool tryGetCanActivateRune(int &can_activate) const;
        int resolveRequestedPosture() const;
        rm_protocol::SentryPosture resolvePostureEnum() const;
        const char *runeTypeName() const;
        const char *runeStatusKey() const;
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
        bool scan_mode_disabled_{false};
        bool yaw_controller_triggered_{false};
        bool auto_shoot_enabled_{false};
        bool saw_activating_state_{false};
    };

    // 动作 4：打前哨站
    // 顺序：scan_mode=false -> yaw_controller=1 -> outpost_mode_type=true
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
        bool publishScanMode(bool enabled);
        bool triggerYawController();
        bool publishOutpostMode(bool enabled);
        // 确保打前哨站所需输出已经按顺序打开；已打开的输出不会重复发布。
        bool ensureEngageOutputs();
        void cleanupOutputs();
        void setOutpostOutcome(bool success, const std::string &result) const;

        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<ControlTopicPublishers> control_publishers_;

        int timeout_ms_{45000};
        std::chrono::steady_clock::time_point start_time_{};
        bool scan_mode_disabled_{false};
        bool yaw_controller_triggered_{false};
        bool outpost_mode_enabled_{false};
    };

} // namespace sentry_nav_bt_test

#endif // SENTRY_NAV_BT_TEST_REFEREE_ACTIONS_HPP_

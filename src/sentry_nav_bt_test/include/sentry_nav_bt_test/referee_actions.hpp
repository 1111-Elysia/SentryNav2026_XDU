#ifndef SENTRY_NAV_BT_TEST_REFEREE_ACTIONS_HPP_
#define SENTRY_NAV_BT_TEST_REFEREE_ACTIONS_HPP_

#include <chrono>
#include <string>
#include <memory>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rm_referee_msgs/srv/tx.hpp"
#include "sentry_msgs/msg/scan_mode.hpp"
#include "sentry_msgs/msg/vw.hpp"
#include "sentry_nav_bt_test/cmd_utils.hpp" 
#include "std_msgs/msg/bool.hpp"

namespace sentry_nav_bt_test
{
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

        bool publishYawController(bool enabled)
        {
            ensureInitialized();
            if (!yaw_controller_pub_) {
                logMissingPublisher("/yaw_controller");
                return false;
            }

            std_msgs::msg::Bool msg;
            msg.data = enabled;
            yaw_controller_pub_->publish(msg);
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
            yaw_controller_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/yaw_controller", 10);
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
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr yaw_controller_pub_;
    };

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

        // 同步发送数据包，并等待 /rm_referee/tx 返回结果
        bool send_packet(
            const std::vector<uint8_t> &data,
            std::chrono::milliseconds response_timeout = std::chrono::milliseconds(200));

        rclcpp::Node::SharedPtr node_;
        rclcpp::Client<rm_referee_msgs::srv::Tx>::SharedPtr client_;
        std::shared_ptr<SentryRefereeUtils> utils_;
    };

    // 动作 1: SetSentryPosture (切换姿态 - 闭环控制)
    // XML: <Action ID="SetSentryPosture" mode="1" timeout_ms="1000"/>
    class SetSentryPosture : public RefereeActionBase
    {
    public:
        SetSentryPosture(const std::string &name, const BT::NodeConfiguration &config);

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;

    private:
        int last_confirmed_mode_{-1};
    };

    // 动作 2: RequestActivateRune 
    // XML: <Action ID="RequestActivateRune" posture="0"/>
    class RequestActivateRune : public RefereeActionBase
    {
    public:
        RequestActivateRune(const std::string &name, const BT::NodeConfiguration &config);

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;
    };

    // 动作 3: ConfirmResurrection
    // XML: <Action ID="ConfirmResurrection" posture="0" burst_count="3" burst_interval_ms="20"/>
    class ConfirmResurrection : public RefereeActionBase
    {
    public:
        ConfirmResurrection(const std::string &name, const BT::NodeConfiguration &config);

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;

    private:
        std::chrono::steady_clock::time_point last_send_time_{};
    };

    class EngageRune : public BT::StatefulActionNode
    {
    public:
        EngageRune(const std::string &name, const BT::NodeConfiguration &config);

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
        void cleanupOutputs();
        bool tryGetRuneStatus(int &status) const;
        bool tryGetCanActivateRune(int &can_activate) const;
        int resolveRequestedPosture() const;
        rm_protocol::SentryPosture resolvePostureEnum() const;
        const char *runeTypeName() const;
        const char *runeStatusKey() const;

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
        bool auto_shoot_enabled_{false};
        bool saw_activating_state_{false};
    };

} // namespace sentry_nav_bt_test

#endif // SENTRY_NAV_BT_TEST_REFEREE_ACTIONS_HPP_

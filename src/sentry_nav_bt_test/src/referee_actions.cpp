#include "sentry_nav_bt_test/referee_actions.hpp"
#include <thread>
#include <chrono>

namespace sentry_nav_bt_test
{
    // ==========================================
    // 基类实现
    // ==========================================
    RefereeActionBase::RefereeActionBase(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
        // 从黑板获取 ROS Node 指针
        if (!config.blackboard->get("node", node_)) {
            throw std::runtime_error("Missing 'node' in blackboard");
        }
        client_ = node_->create_client<rm_referee_msgs::srv::Tx>("/rm_referee/tx");
    }

    bool RefereeActionBase::initUtils()
    {
        if (utils_) return true; 

        uint8_t robot_id = 0;
        // 尝试从黑板获取 Robot ID
        if (config().blackboard->get("robot_id", robot_id) && robot_id != 0) {
            utils_ = std::make_shared<SentryRefereeUtils>(robot_id);
            // RCLCPP_INFO(node_->get_logger(), "Referee Utils Initialized with ID: %d", robot_id);
            return true;
        }
        return false;
    }

    void RefereeActionBase::send_packet(const std::vector<uint8_t> &data)
    {
        if (!client_->service_is_ready()) return; 
        auto request = std::make_shared<rm_referee_msgs::srv::Tx::Request>();
        request->data = data;
        client_->async_send_request(request);
    }

    // ==========================================
    // SetSentryPosture 实现 
    // ==========================================
    SetSentryPosture::SetSentryPosture(const std::string &name, const BT::NodeConfiguration &config)
        : RefereeActionBase(name, config) {}

    BT::PortsList SetSentryPosture::providedPorts() {
        return {
            BT::InputPort<int>("mode", "1:Attack, 2:Defend, 3:Move"),
            BT::InputPort<int>("timeout_ms", 1000, "超时时间(ms)")
        };
    }

    BT::NodeStatus SetSentryPosture::tick()
    {
        if (!initUtils()) return BT::NodeStatus::FAILURE;

        // 1. 获取输入参数
        int target_mode_int;
        int timeout_ms;
        if (!getInput("mode", target_mode_int)) return BT::NodeStatus::FAILURE;
        getInput("timeout_ms", timeout_ms);

        // 2. 闭环检查循环
        auto start_time = std::chrono::steady_clock::now();
        
        while (true) {
            // [Check] 检查是否超时
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count() > timeout_ms) {
                RCLCPP_WARN(node_->get_logger(), "SetSentryPosture 超时! 目标: %d", target_mode_int);
                return BT::NodeStatus::FAILURE; 
            }

            // [Feedback] 从黑板读取当前真实姿态
            int current_real_posture = -1;
            config().blackboard->get("current_posture", current_real_posture);

            // [Success Condition] 目标达成
            if (current_real_posture == target_mode_int) {
                RCLCPP_INFO(node_->get_logger(), "SetSentryPosture 成功! 当前姿态: %d", current_real_posture);
                return BT::NodeStatus::SUCCESS;
            }

            // [Action] 发送指令 (activate_energy=false)
            auto posture_enum = static_cast<rm_protocol::SentryPosture>(target_mode_int);
            // 注意：buildSentryCmdPacket 需要在 cmd_utils.hpp 中定义
            auto packet = utils_->buildSentryCmdPacket(posture_enum, false);
            send_packet(packet);

            // [Wait] 等待一小段时间再重试/检查 (防止发包太快阻塞串口)
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    // ==========================================
    // RequestActivateRune 实现
    // ==========================================
    RequestActivateRune::RequestActivateRune(const std::string &name, const BT::NodeConfiguration &config)
        : RefereeActionBase(name, config) {}

    BT::PortsList RequestActivateRune::providedPorts() {
        return {
            BT::InputPort<int>("posture", 0, "请求时保持的姿态 (0表示自动读取当前姿态)")
        };
    }

    BT::NodeStatus RequestActivateRune::tick()
    {
        if (!initUtils()) return BT::NodeStatus::FAILURE;

        int req_posture_int = 0;
        getInput("posture", req_posture_int);

        // 1. 确定发送时的姿态 (State Protection)
        // 如果 XML 没指定姿态(0)，则从黑板读取当前姿态，防止开符时姿态被重置
        if (req_posture_int == 0) {
            int current_val = 3; // 默认 Move
            if (config().blackboard->get("current_posture", current_val)) {
                req_posture_int = current_val;
            } else {
                req_posture_int = 3; 
            }
        }

        // 2. 鲁棒发送 (Robust Send)
        auto posture_enum = static_cast<rm_protocol::SentryPosture>(req_posture_int);
        // activate_energy=true
        auto packet = utils_->buildSentryCmdPacket(posture_enum, true); 

        RCLCPP_INFO(node_->get_logger(), ">>> 请求激活能量机关 (保持姿态: %d) <<<", req_posture_int);

        // 连发 5 次确保送达
        for (int i = 0; i < 5; ++i) {
            send_packet(packet);
            std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 间隔20ms
        }

        // 直接返回成功，后续由行为树的 <Wait> 节点等待
        return BT::NodeStatus::SUCCESS;
    }

} // namespace sentry_nav_bt
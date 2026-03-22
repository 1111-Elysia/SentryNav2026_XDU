#include "sentry_nav_bt_test/referee_actions.hpp"

#include <chrono>
#include <thread>

namespace sentry_nav_bt_test
{

RefereeActionBase::RefereeActionBase(const std::string &name, const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config)
{
    if (!config.blackboard->get("node", node_)) {
        throw std::runtime_error("Missing 'node' in blackboard");
    }
    client_ = node_->create_client<rm_referee_msgs::srv::Tx>("/rm_referee/tx");
}

bool RefereeActionBase::initUtils()
{
    if (utils_) {
        return true;
    }

    uint8_t robot_id = 0;
    if (config().blackboard->get("robot_id", robot_id) && robot_id != 0) {
        utils_ = std::make_shared<SentryRefereeUtils>(robot_id);
        return true;
    }
    return false;
}

void RefereeActionBase::send_packet(const std::vector<uint8_t> &data)
{
    if (!client_->service_is_ready()) {
        return;
    }
    auto request = std::make_shared<rm_referee_msgs::srv::Tx::Request>();
    request->data = data;
    client_->async_send_request(request);
}

SetSentryPosture::SetSentryPosture(const std::string &name, const BT::NodeConfiguration &config)
    : RefereeActionBase(name, config)
{
}

BT::PortsList SetSentryPosture::providedPorts()
{
    return {
        BT::InputPort<int>("mode", "1:Attack, 2:Defend, 3:Move"),
        BT::InputPort<int>("timeout_ms", 1000, "超时时间(ms)")};
}

BT::NodeStatus SetSentryPosture::tick()
{
    if (!initUtils()) {
        return BT::NodeStatus::FAILURE;
    }

    int target_mode_int;
    int timeout_ms;
    if (!getInput("mode", target_mode_int)) {
        return BT::NodeStatus::FAILURE;
    }
    getInput("timeout_ms", timeout_ms);

    auto start_time = std::chrono::steady_clock::now();

    while (true) {
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count() > timeout_ms) {
            RCLCPP_WARN(node_->get_logger(), "SetSentryPosture 超时! 目标: %d", target_mode_int);
            return BT::NodeStatus::FAILURE;
        }

        int current_real_posture = -1;
        config().blackboard->get("current_posture", current_real_posture);

        if (current_real_posture == target_mode_int) {
            RCLCPP_INFO(node_->get_logger(), "SetSentryPosture 成功! 当前姿态: %d", current_real_posture);
            return BT::NodeStatus::SUCCESS;
        }

        auto posture_enum = static_cast<rm_protocol::SentryPosture>(target_mode_int);
        auto packet = utils_->buildSentryCmdPacket(posture_enum, false);
        send_packet(packet);

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

RequestActivateRune::RequestActivateRune(const std::string &name, const BT::NodeConfiguration &config)
    : RefereeActionBase(name, config)
{
}

BT::PortsList RequestActivateRune::providedPorts()
{
    return {
        BT::InputPort<int>("posture", 0, "请求时保持的姿态 (0表示自动读取当前姿态)")};
}

BT::NodeStatus RequestActivateRune::tick()
{
    if (!initUtils()) {
        return BT::NodeStatus::FAILURE;
    }

    int req_posture_int = 0;
    getInput("posture", req_posture_int);

    if (req_posture_int == 0) {
        int current_val = 3;
        if (config().blackboard->get("current_posture", current_val)) {
            req_posture_int = current_val;
        } else {
            req_posture_int = 3;
        }
    }

    auto posture_enum = static_cast<rm_protocol::SentryPosture>(req_posture_int);
    auto packet = utils_->buildSentryCmdPacket(posture_enum, true);

    RCLCPP_INFO(node_->get_logger(), ">>> 请求激活能量机关 (保持姿态: %d) <<<", req_posture_int);

    for (int i = 0; i < 5; ++i) {
        send_packet(packet);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    return BT::NodeStatus::SUCCESS;
}

ConfirmResurrection::ConfirmResurrection(const std::string &name, const BT::NodeConfiguration &config)
    : RefereeActionBase(name, config)
{
}

BT::PortsList ConfirmResurrection::providedPorts()
{
    return {
        BT::InputPort<int>("posture", 0, "确认复活时保持的姿态 (0表示自动读取当前姿态)"),
        BT::InputPort<int>("burst_count", 3, "每次tick连发次数"),
        BT::InputPort<int>("burst_interval_ms", 20, "连发间隔(ms)")};
}

BT::NodeStatus ConfirmResurrection::tick()
{
    if (!initUtils()) {
        return BT::NodeStatus::FAILURE;
    }

    int req_posture_int = 0;
    int burst_count = 3;
    int burst_interval_ms = 20;
    getInput("posture", req_posture_int);
    getInput("burst_count", burst_count);
    getInput("burst_interval_ms", burst_interval_ms);

    if (burst_count < 1) {
        burst_count = 1;
    }
    if (burst_interval_ms < 0) {
        burst_interval_ms = 0;
    }

    if (req_posture_int == 0) {
        int current_val = 3;
        if (config().blackboard->get("current_posture", current_val)) {
            req_posture_int = current_val;
        } else {
            req_posture_int = 3;
        }
    }

    auto posture_enum = static_cast<rm_protocol::SentryPosture>(req_posture_int);
    auto packet = utils_->buildSentryCmdPacket(posture_enum, false, true);

    for (int i = 0; i < burst_count; ++i) {
        send_packet(packet);
        if (burst_interval_ms > 0 && i + 1 < burst_count) {
            std::this_thread::sleep_for(std::chrono::milliseconds(burst_interval_ms));
        }
    }

    return BT::NodeStatus::SUCCESS;
}

} // namespace sentry_nav_bt_test

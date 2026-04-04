#include "sentry_nav_bt_test/referee_actions.hpp"

#include <chrono>
#include <thread>

namespace sentry_nav_bt_test
{

namespace
{

bool getBlackboardIntLike(const BT::Blackboard::Ptr &blackboard, const std::string &key, int &value)
{
    if (!blackboard) {
        return false;
    }

    if (blackboard->get(key, value)) {
        return true;
    }

    uint8_t value_u8 = 0;
    if (blackboard->get(key, value_u8)) {
        value = static_cast<int>(value_u8);
        return true;
    }

    uint16_t value_u16 = 0;
    if (blackboard->get(key, value_u16)) {
        value = static_cast<int>(value_u16);
        return true;
    }

    bool value_bool = false;
    if (blackboard->get(key, value_bool)) {
        value = value_bool ? 1 : 0;
        return true;
    }

    return false;
}

bool getBlackboardDoubleLike(const BT::Blackboard::Ptr &blackboard, const std::string &key, double &value)
{
    if (!blackboard) {
        return false;
    }

    if (blackboard->get(key, value)) {
        return true;
    }

    float value_f32 = 0.0f;
    if (blackboard->get(key, value_f32)) {
        value = static_cast<double>(value_f32);
        return true;
    }

    int value_int = 0;
    if (blackboard->get(key, value_int)) {
        value = static_cast<double>(value_int);
        return true;
    }

    return false;
}

} // namespace

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

bool RefereeActionBase::send_packet(
    const std::vector<uint8_t> &data,
    std::chrono::milliseconds response_timeout)
{
    if (!client_->service_is_ready()) {
        if (config().blackboard) {
            config().blackboard->set("last_referee_tx_ok", false);
        }
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            1000,
            "Tx.srv 未就绪，无法发送裁判系统数据");
        return false;
    }

    auto request = std::make_shared<rm_referee_msgs::srv::Tx::Request>();
    request->header.stamp = node_->now();
    request->data = data;

    auto future = client_->async_send_request(request);
    const auto future_status =
        rclcpp::spin_until_future_complete(node_, future, response_timeout);

    bool ok = false;
    if (future_status != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_WARN(
            node_->get_logger(),
            "Tx.srv 调用超时或被中断，data_size=%zu",
            data.size());
    } else {
        try {
            auto response = future.get();
            ok = response && response->ok;
        } catch (const std::exception &e) {
            RCLCPP_WARN(node_->get_logger(), "Tx.srv 调用异常: %s", e.what());
        }
    }

    if (config().blackboard) {
        config().blackboard->set("last_referee_tx_ok", ok);
    }

    if (!ok) {
        RCLCPP_WARN(node_->get_logger(), "Tx.srv 返回失败，data_size=%zu", data.size());
    } else {
        RCLCPP_DEBUG(node_->get_logger(), "Tx.srv 返回成功，data_size=%zu", data.size());
    }

    return ok;
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
    bool sent_command_this_tick = false;

    while (true) {
        rclcpp::spin_some(node_);

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count() > timeout_ms) {
            RCLCPP_WARN(node_->get_logger(), "SetSentryPosture 超时! 目标: %d", target_mode_int);
            return BT::NodeStatus::FAILURE;
        }

        int current_real_posture = -1;
        if (getBlackboardIntLike(config().blackboard, "current_posture", current_real_posture)) {
            if (current_real_posture == target_mode_int) {
                if (last_confirmed_mode_ != target_mode_int) {
                    if (sent_command_this_tick) {
                        RCLCPP_INFO(node_->get_logger(), "SetSentryPosture 成功! 当前姿态: %d", current_real_posture);
                    } else {
                        RCLCPP_INFO(node_->get_logger(), "SetSentryPosture 跳过发送，当前已是目标姿态: %d", current_real_posture);
                    }
                } else if (sent_command_this_tick) {
                    RCLCPP_INFO(node_->get_logger(), "SetSentryPosture 成功! 当前姿态: %d", current_real_posture);
                }
                last_confirmed_mode_ = target_mode_int;
                return BT::NodeStatus::SUCCESS;
            }
            last_confirmed_mode_ = -1;
        }

        auto posture_enum = static_cast<rm_protocol::SentryPosture>(target_mode_int);
        auto packet = utils_->buildSentryCmdPacket(posture_enum, false);
        send_packet(packet);
        sent_command_this_tick = true;

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
        if (getBlackboardIntLike(config().blackboard, "current_posture", current_val)) {
            req_posture_int = current_val;
        } else {
            req_posture_int = 3;
        }
    }

    auto posture_enum = static_cast<rm_protocol::SentryPosture>(req_posture_int);
    auto packet = utils_->buildSentryCmdPacket(posture_enum, true);

    RCLCPP_INFO(node_->get_logger(), ">>> 请求激活能量机关 (保持姿态: %d) <<<", req_posture_int);

    bool any_send_ok = false;
    for (int i = 0; i < 5; ++i) {
        any_send_ok = send_packet(packet) || any_send_ok;
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    return any_send_ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
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
        BT::InputPort<int>("burst_interval_ms", 20, "连发间隔(ms)"),
        BT::InputPort<int>("min_interval_ms", 100, "两次确认复活发送之间的最小间隔(ms)")};
}

BT::NodeStatus ConfirmResurrection::tick()
{
    if (!initUtils()) {
        return BT::NodeStatus::FAILURE;
    }

    uint16_t current_hp = 0;
    if (config().blackboard->get("current_hp", current_hp) && current_hp > 0U) {
        return BT::NodeStatus::SUCCESS;
    }

    bool can_confirm_resurrection = false;
    if (!config().blackboard->get("can_confirm_resurrection", can_confirm_resurrection) ||
        !can_confirm_resurrection) {
        return BT::NodeStatus::SUCCESS;
    }

    int req_posture_int = 0;
    int burst_count = 3;
    int burst_interval_ms = 20;
    int min_interval_ms = 100;
    getInput("posture", req_posture_int);
    getInput("burst_count", burst_count);
    getInput("burst_interval_ms", burst_interval_ms);
    getInput("min_interval_ms", min_interval_ms);

    if (burst_count < 1) {
        burst_count = 1;
    }
    if (burst_interval_ms < 0) {
        burst_interval_ms = 0;
    }
    if (min_interval_ms < 0) {
        min_interval_ms = 0;
    }

    const auto now_tp = std::chrono::steady_clock::now();
    if (last_send_time_.time_since_epoch().count() != 0 &&
        std::chrono::duration_cast<std::chrono::milliseconds>(now_tp - last_send_time_).count() < min_interval_ms) {
        return BT::NodeStatus::SUCCESS;
    }

    if (req_posture_int == 0) {
        int current_val = 3;
        if (getBlackboardIntLike(config().blackboard, "current_posture", current_val)) {
            req_posture_int = current_val;
        } else {
            req_posture_int = 3;
        }
    }

    auto posture_enum = static_cast<rm_protocol::SentryPosture>(req_posture_int);
    auto packet = utils_->buildSentryCmdPacket(posture_enum, false, true);

    bool any_send_ok = false;
    last_send_time_ = now_tp;
    for (int i = 0; i < burst_count; ++i) {
        any_send_ok = send_packet(packet) || any_send_ok;
        rclcpp::spin_some(node_);
        if (burst_interval_ms > 0 && i + 1 < burst_count) {
            std::this_thread::sleep_for(std::chrono::milliseconds(burst_interval_ms));
        }
    }

    return any_send_ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

EngageRune::EngageRune(const std::string &name, const BT::NodeConfiguration &config)
    : BT::StatefulActionNode(name, config)
{
    if (!config.blackboard->get("node", node_)) {
        throw std::runtime_error("Missing 'node' in blackboard");
    }

    client_ = node_->create_client<rm_referee_msgs::srv::Tx>("/rm_referee/tx");
    control_publishers_ = std::make_shared<ControlTopicPublishers>(node_);
}

BT::PortsList EngageRune::providedPorts()
{
    return {
        BT::InputPort<std::string>("rune_type", "small", "small 或 large"),
        BT::InputPort<int>("posture", 0, "打符时保持的姿态 (0 表示自动读取当前姿态)"),
        BT::InputPort<int>("timeout_ms", 45000, "整段打符流程的超时时间"),
        BT::InputPort<int>("request_interval_ms", 1000, "重复发送 0x0120 bit23 的最小间隔")};
}

bool EngageRune::initUtils()
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

bool EngageRune::send_packet(
    const std::vector<uint8_t> &data,
    std::chrono::milliseconds response_timeout)
{
    if (!client_->service_is_ready()) {
        if (config().blackboard) {
            config().blackboard->set("last_referee_tx_ok", false);
        }
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            1000,
            "Tx.srv 未就绪，无法发送裁判系统数据");
        return false;
    }

    auto request = std::make_shared<rm_referee_msgs::srv::Tx::Request>();
    request->header.stamp = node_->now();
    request->data = data;

    auto future = client_->async_send_request(request);
    const auto future_status =
        rclcpp::spin_until_future_complete(node_, future, response_timeout);

    bool ok = false;
    if (future_status != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_WARN(
            node_->get_logger(),
            "Tx.srv 调用超时或被中断，data_size=%zu",
            data.size());
    } else {
        try {
            auto response = future.get();
            ok = response && response->ok;
        } catch (const std::exception &e) {
            RCLCPP_WARN(node_->get_logger(), "Tx.srv 调用异常: %s", e.what());
        }
    }

    if (config().blackboard) {
        config().blackboard->set("last_referee_tx_ok", ok);
    }

    if (!ok) {
        RCLCPP_WARN(node_->get_logger(), "Tx.srv 返回失败，data_size=%zu", data.size());
    }

    return ok;
}

bool EngageRune::publishScanMode(bool enabled)
{
    return control_publishers_ && control_publishers_->publishScanMode(enabled);
}

bool EngageRune::publishAutoShoot(bool enabled)
{
    return control_publishers_ && control_publishers_->publishAutoShoot(enabled);
}

bool EngageRune::triggerYawController()
{
    return control_publishers_ && control_publishers_->publishYawController(true);
}

void EngageRune::cleanupOutputs()
{
    if (config().blackboard) {
        config().blackboard->set("in_rune_phase", 0);
    }

    if (auto_shoot_enabled_) {
        publishAutoShoot(false);
        auto_shoot_enabled_ = false;
    }

    if (scan_mode_disabled_) {
        publishScanMode(true);
        scan_mode_disabled_ = false;
    }
}

bool EngageRune::tryGetRuneStatus(int &status) const
{
    return getBlackboardIntLike(config().blackboard, runeStatusKey(), status);
}

bool EngageRune::tryGetCanActivateRune(int &can_activate) const
{
    return getBlackboardIntLike(config().blackboard, "can_activate_rune", can_activate);
}

int EngageRune::resolveRequestedPosture() const
{
    if (requested_posture_ != 0) {
        return requested_posture_;
    }

    int current_val = 3;
    if (getBlackboardIntLike(config().blackboard, "current_posture", current_val) &&
        current_val >= 1 && current_val <= 3) {
        return current_val;
    }
    return 3;
}

rm_protocol::SentryPosture EngageRune::resolvePostureEnum() const
{
    return static_cast<rm_protocol::SentryPosture>(resolveRequestedPosture());
}

const char *EngageRune::runeTypeName() const
{
    return rune_type_ == RuneType::SMALL ? "小能量机关" : "大能量机关";
}

const char *EngageRune::runeStatusKey() const
{
    return rune_type_ == RuneType::SMALL ? "small_rune_status" : "large_rune_status";
}

BT::NodeStatus EngageRune::onStart()
{
    if (!initUtils()) {
        RCLCPP_WARN(node_->get_logger(), "EngageRune: robot_id 尚未就绪");
        return BT::NodeStatus::FAILURE;
    }

    std::string rune_type = "small";
    getInput("rune_type", rune_type);
    if (rune_type == "small") {
        rune_type_ = RuneType::SMALL;
    } else if (rune_type == "large" || rune_type == "big") {
        rune_type_ = RuneType::LARGE;
    } else {
        RCLCPP_ERROR(node_->get_logger(), "EngageRune: 不支持的 rune_type=%s", rune_type.c_str());
        return BT::NodeStatus::FAILURE;
    }

    getInput("posture", requested_posture_);
    getInput("timeout_ms", timeout_ms_);
    getInput("request_interval_ms", request_interval_ms_);

    if (timeout_ms_ < 1) {
        timeout_ms_ = 1;
    }
    if (request_interval_ms_ < 0) {
        request_interval_ms_ = 0;
    }

    start_time_ = std::chrono::steady_clock::now();
    last_request_time_ = std::chrono::steady_clock::time_point{};
    saw_activating_state_ = false;
    auto_shoot_enabled_ = false;
    if (config().blackboard) {
        config().blackboard->set("in_rune_phase", 1);
    }

    publishAutoShoot(false);
    if (publishScanMode(false)) {
        scan_mode_disabled_ = true;
    } else {
        scan_mode_disabled_ = false;
    }
    triggerYawController();

    RCLCPP_INFO(
        node_->get_logger(),
        "EngageRune: 进入%s流程，先关闭 scan mode，再向 /yaw_controller 发送一次 true",
        runeTypeName());

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus EngageRune::onRunning()
{
    rclcpp::spin_some(node_);

    const auto now_tp = std::chrono::steady_clock::now();
    const auto elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now_tp - start_time_).count();
    if (elapsed_ms > timeout_ms_) {
        RCLCPP_WARN(node_->get_logger(), "EngageRune: %s流程超时", runeTypeName());
        cleanupOutputs();
        return BT::NodeStatus::FAILURE;
    }

    int rune_status = 0;
    if (!tryGetRuneStatus(rune_status)) {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            1000,
            "EngageRune: 尚未获取 %s 状态键 %s",
            runeTypeName(),
            runeStatusKey());
        return BT::NodeStatus::RUNNING;
    }

    if (rune_status == 2) {
        if (!saw_activating_state_) {
            saw_activating_state_ = true;
            RCLCPP_INFO(node_->get_logger(), "EngageRune: %s进入正在激活状态", runeTypeName());
        }
        if (!auto_shoot_enabled_ && publishAutoShoot(true)) {
            auto_shoot_enabled_ = true;
            RCLCPP_INFO(node_->get_logger(), "EngageRune: 已向 autoshoot 发送 true");
        }
        return BT::NodeStatus::RUNNING;
    }

    if (saw_activating_state_ && (rune_status == 0 || rune_status == 1)) {
        RCLCPP_INFO(
            node_->get_logger(),
            "EngageRune: %s流程结束，当前状态=%d，恢复 scan mode 并关闭 autoshoot",
            runeTypeName(),
            rune_status);
        cleanupOutputs();
        return BT::NodeStatus::SUCCESS;
    }

    if (rune_status == 1) {
        RCLCPP_INFO(
            node_->get_logger(),
            "EngageRune: %s已处于已激活状态，无需继续请求激活",
            runeTypeName());
        cleanupOutputs();
        return BT::NodeStatus::SUCCESS;
    }

    int can_activate_rune = 0;
    if (!tryGetCanActivateRune(can_activate_rune) || can_activate_rune != 1) {
        return BT::NodeStatus::RUNNING;
    }

    const bool interval_elapsed =
        last_request_time_.time_since_epoch().count() == 0 ||
        std::chrono::duration_cast<std::chrono::milliseconds>(now_tp - last_request_time_).count() >=
            request_interval_ms_;
    if (!interval_elapsed) {
        return BT::NodeStatus::RUNNING;
    }

    auto packet = utils_->buildSentryCmdPacket(resolvePostureEnum(), true);
    last_request_time_ = now_tp;
    if (send_packet(packet)) {
        RCLCPP_INFO(
            node_->get_logger(),
            "EngageRune: 已发送 0x0120 bit23，请求%s进入正在激活状态",
            runeTypeName());
    }

    return BT::NodeStatus::RUNNING;
}

void EngageRune::onHalted()
{
    cleanupOutputs();
}

} // namespace sentry_nav_bt_test

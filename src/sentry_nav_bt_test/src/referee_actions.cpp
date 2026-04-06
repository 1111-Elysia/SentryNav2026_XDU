#include "sentry_nav_bt_test/referee_actions.hpp"

#include <algorithm>
#include <chrono>
#include <thread>

namespace sentry_nav_bt_test
{

namespace
{

constexpr const char *kLastPostureRequestTargetKey = "last_posture_request_target";
constexpr const char *kLastPostureRequestSentKey = "last_posture_request_sent";
constexpr const char *kLastPostureRequestTxOkKey = "last_posture_request_tx_ok";
constexpr const char *kLastPostureRequestConfirmedKey = "last_posture_request_confirmed";
constexpr const char *kLastPostureRequestPendingKey = "last_posture_request_pending";
constexpr const char *kLastPostureRequestResultKey = "last_posture_request_result";
constexpr const char *kLastPostureRequestTimeKey = "last_posture_request_time_s";
constexpr const char *kPostureSwitchCooldownKey = "posture_switch_cooldown_ms";

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

double steadyNowSeconds()
{
    using seconds_f64 = std::chrono::duration<double>;
    return std::chrono::duration_cast<seconds_f64>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

void updatePostureRequestStatus(
    const BT::Blackboard::Ptr &blackboard,
    int target_mode,
    bool sent,
    bool tx_ok,
    bool confirmed,
    bool pending,
    const std::string &result,
    double request_time_s)
{
    if (!blackboard) {
        return;
    }

    blackboard->set(kLastPostureRequestTargetKey, target_mode);
    blackboard->set(kLastPostureRequestSentKey, sent);
    blackboard->set(kLastPostureRequestTxOkKey, tx_ok);
    blackboard->set(kLastPostureRequestConfirmedKey, confirmed);
    blackboard->set(kLastPostureRequestPendingKey, pending);
    blackboard->set(kLastPostureRequestResultKey, result);
    blackboard->set(kLastPostureRequestTimeKey, request_time_s);
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

void SetSentryPosture::updateRequestStatus(
    int target_mode,
    bool sent,
    bool tx_ok,
    bool confirmed,
    bool pending,
    const std::string &result,
    double request_time_s) const
{
    updatePostureRequestStatus(
        config().blackboard,
        target_mode,
        sent,
        tx_ok,
        confirmed,
        pending,
        result,
        request_time_s);
}

BT::PortsList SetSentryPosture::providedPorts()
{
    return {
        BT::InputPort<int>("mode", "1:Attack, 2:Defend, 3:Move"),
        BT::InputPort<int>("timeout_ms", 1000, "首次发送后短暂等待姿态回显(ms)"),
        BT::InputPort<int>("cooldown_ms", 5000, "全局姿态切换冷却时间(ms)，冷却内不重复发同类请求")};
}

BT::NodeStatus SetSentryPosture::tick()
{
    int target_mode_int;
    int timeout_ms;
    int cooldown_ms;
    if (!getInput("mode", target_mode_int)) {
        return BT::NodeStatus::FAILURE;
    }
    getInput("timeout_ms", timeout_ms);
    getInput("cooldown_ms", cooldown_ms);

    if (config().blackboard) {
        int bb_cooldown_ms = cooldown_ms;
        if (getBlackboardIntLike(config().blackboard, kPostureSwitchCooldownKey, bb_cooldown_ms)) {
            cooldown_ms = bb_cooldown_ms;
        }
    }

    timeout_ms = std::max(timeout_ms, 0);
    cooldown_ms = std::max(cooldown_ms, 0);
    const double now_s = steadyNowSeconds();

    if (!initUtils()) {
        updateRequestStatus(
            target_mode_int,
            false,
            false,
            false,
            false,
            "skipped_robot_id_unavailable",
            now_s);
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            1000,
            "SetSentryPosture 跳过: robot_id 尚未就绪，目标姿态: %d",
            target_mode_int);
        return BT::NodeStatus::SUCCESS;
    }

    int current_real_posture = -1;
    if (getBlackboardIntLike(config().blackboard, "current_posture", current_real_posture) &&
        current_real_posture == target_mode_int) {
        if (last_confirmed_mode_ != target_mode_int) {
            RCLCPP_INFO(node_->get_logger(), "SetSentryPosture 跳过发送，当前已是目标姿态: %d", current_real_posture);
        }
        last_confirmed_mode_ = target_mode_int;
        updateRequestStatus(target_mode_int, false, true, true, false, "already_target", now_s);
        return BT::NodeStatus::SUCCESS;
    }
    last_confirmed_mode_ = -1;

    int last_target_mode = -1;
    bool last_tx_ok = false;
    bool last_pending = false;
    double last_request_time_s = -1.0;
    if (config().blackboard) {
        getBlackboardIntLike(config().blackboard, kLastPostureRequestTargetKey, last_target_mode);
        config().blackboard->get(kLastPostureRequestTxOkKey, last_tx_ok);
        config().blackboard->get(kLastPostureRequestPendingKey, last_pending);
        getBlackboardDoubleLike(config().blackboard, kLastPostureRequestTimeKey, last_request_time_s);
    }

    if (last_pending &&
        current_real_posture >= 1 && current_real_posture <= 3 &&
        current_real_posture == last_target_mode) {
        updateRequestStatus(
            last_target_mode,
            false,
            true,
            true,
            false,
            "confirmed_from_feedback",
            last_request_time_s > 0.0 ? last_request_time_s : now_s);
        last_pending = false;
        last_tx_ok = true;
    }

    const bool cooldown_active =
        last_request_time_s > 0.0 &&
        (now_s - last_request_time_s) * 1000.0 < static_cast<double>(cooldown_ms);
    if (cooldown_active) {
        const double remain_ms =
            std::max(0.0, static_cast<double>(cooldown_ms) - (now_s - last_request_time_s) * 1000.0);
        if (last_target_mode == target_mode_int) {
            updateRequestStatus(
                target_mode_int,
                false,
                last_tx_ok,
                false,
                true,
                last_tx_ok ? "pending_same_target_cooldown" : "retry_backoff_same_target",
                last_request_time_s);
            RCLCPP_INFO_THROTTLE(
                node_->get_logger(),
                *node_->get_clock(),
                1000,
                "SetSentryPosture 冷却中: 姿态 %d 已请求，%.0f ms 后才允许重发",
                target_mode_int,
                remain_ms);
            return BT::NodeStatus::SUCCESS;
        }

        updateRequestStatus(
            target_mode_int,
            false,
            last_tx_ok,
            false,
            false,
            "cooldown_blocked_by_previous_target",
            last_request_time_s);
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            1000,
            "SetSentryPosture 冷却中: 上次请求姿态 %d，本次目标姿态 %d 暂不重发，剩余 %.0f ms",
            last_target_mode,
            target_mode_int,
            remain_ms);
        return BT::NodeStatus::SUCCESS;
    }

    auto posture_enum = static_cast<rm_protocol::SentryPosture>(target_mode_int);
    auto packet = utils_->buildSentryCmdPacket(posture_enum, false);
    const bool tx_ok = send_packet(packet);
    updateRequestStatus(
        target_mode_int,
        true,
        tx_ok,
        false,
        true,
        tx_ok ? "sent_waiting_confirmation" : "tx_failed_backoff",
        now_s);

    if (!tx_ok) {
        RCLCPP_WARN(
            node_->get_logger(),
            "SetSentryPosture 发送失败，目标姿态: %d，进入冷却回退避免重复重发",
            target_mode_int);
        return BT::NodeStatus::SUCCESS;
    }

    const auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - start_time)
               .count() <= timeout_ms) {
        rclcpp::spin_some(node_);

        if (getBlackboardIntLike(config().blackboard, "current_posture", current_real_posture) &&
            current_real_posture == target_mode_int) {
            RCLCPP_INFO(node_->get_logger(), "SetSentryPosture 成功! 当前姿态: %d", current_real_posture);
            last_confirmed_mode_ = target_mode_int;
            updateRequestStatus(target_mode_int, true, true, true, false, "confirmed", now_s);
            return BT::NodeStatus::SUCCESS;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    RCLCPP_INFO(
        node_->get_logger(),
        "SetSentryPosture 已发送姿态 %d，请等待裁判系统回显；冷却 %d ms 内不会重复发送",
        target_mode_int,
        cooldown_ms);
    return BT::NodeStatus::SUCCESS;
}

MaintainSentryPosture::MaintainSentryPosture(const std::string &name, const BT::NodeConfiguration &config)
    : RefereeActionBase(name, config)
{
}

BT::PortsList MaintainSentryPosture::providedPorts()
{
    return {
        BT::InputPort<int>("mode", "1:Attack, 2:Defend, 3:Move"),
        BT::InputPort<int>("cooldown_ms", 5000, "全局姿态切换冷却时间(ms)，冷却内不重复发同类请求")};
}

BT::NodeStatus MaintainSentryPosture::tick()
{
    int target_mode_int;
    int cooldown_ms;
    if (!getInput("mode", target_mode_int)) {
        return BT::NodeStatus::FAILURE;
    }
    getInput("cooldown_ms", cooldown_ms);

    if (config().blackboard) {
        int bb_cooldown_ms = cooldown_ms;
        if (getBlackboardIntLike(config().blackboard, kPostureSwitchCooldownKey, bb_cooldown_ms)) {
            cooldown_ms = bb_cooldown_ms;
        }
    }

    cooldown_ms = std::max(cooldown_ms, 0);
    const double now_s = steadyNowSeconds();

    if (!initUtils()) {
        updatePostureRequestStatus(
            config().blackboard,
            target_mode_int,
            false,
            false,
            false,
            false,
            "skipped_robot_id_unavailable",
            now_s);
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            1000,
            "MaintainSentryPosture 跳过: robot_id 尚未就绪，目标姿态: %d",
            target_mode_int);
        return BT::NodeStatus::SUCCESS;
    }

    int current_real_posture = -1;
    if (getBlackboardIntLike(config().blackboard, "current_posture", current_real_posture) &&
        current_real_posture == target_mode_int) {
        if (last_confirmed_mode_ != target_mode_int) {
            RCLCPP_INFO(
                node_->get_logger(),
                "MaintainSentryPosture 已确认当前姿态为目标值: %d",
                current_real_posture);
        }
        last_confirmed_mode_ = target_mode_int;
        updatePostureRequestStatus(
            config().blackboard,
            target_mode_int,
            false,
            true,
            true,
            false,
            "already_target",
            now_s);
        return BT::NodeStatus::SUCCESS;
    }
    last_confirmed_mode_ = -1;

    int last_target_mode = -1;
    bool last_tx_ok = false;
    bool last_pending = false;
    double last_request_time_s = -1.0;
    if (config().blackboard) {
        getBlackboardIntLike(config().blackboard, kLastPostureRequestTargetKey, last_target_mode);
        config().blackboard->get(kLastPostureRequestTxOkKey, last_tx_ok);
        config().blackboard->get(kLastPostureRequestPendingKey, last_pending);
        getBlackboardDoubleLike(config().blackboard, kLastPostureRequestTimeKey, last_request_time_s);
    }

    if (last_pending &&
        current_real_posture >= 1 && current_real_posture <= 3 &&
        current_real_posture == last_target_mode) {
        updatePostureRequestStatus(
            config().blackboard,
            last_target_mode,
            false,
            true,
            true,
            false,
            "confirmed_from_feedback",
            last_request_time_s > 0.0 ? last_request_time_s : now_s);
        last_pending = false;
        last_tx_ok = true;
    }

    const bool cooldown_active =
        last_request_time_s > 0.0 &&
        (now_s - last_request_time_s) * 1000.0 < static_cast<double>(cooldown_ms);
    if (cooldown_active) {
        if (last_target_mode == target_mode_int) {
            updatePostureRequestStatus(
                config().blackboard,
                target_mode_int,
                false,
                last_tx_ok,
                false,
                true,
                last_tx_ok ? "pending_same_target_cooldown" : "retry_backoff_same_target",
                last_request_time_s);
        } else {
            updatePostureRequestStatus(
                config().blackboard,
                target_mode_int,
                false,
                last_tx_ok,
                false,
                false,
                "cooldown_blocked_by_previous_target",
                last_request_time_s);
        }
        return BT::NodeStatus::SUCCESS;
    }

    auto posture_enum = static_cast<rm_protocol::SentryPosture>(target_mode_int);
    auto packet = utils_->buildSentryCmdPacket(posture_enum, false);
    const bool tx_ok = send_packet(packet);
    updatePostureRequestStatus(
        config().blackboard,
        target_mode_int,
        true,
        tx_ok,
        false,
        true,
        tx_ok ? "sent_waiting_confirmation" : "tx_failed_backoff",
        now_s);

    if (tx_ok) {
        RCLCPP_INFO(
            node_->get_logger(),
            "MaintainSentryPosture 已发送姿态 %d，将继续观察回显并按冷却重试",
            target_mode_int);
    } else {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            1000,
            "MaintainSentryPosture 发送姿态 %d 失败，将在后续 tick 中继续重试",
            target_mode_int);
    }

    return BT::NodeStatus::SUCCESS;
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

void EngageRune::setRuneOutcome(bool success, const std::string &result) const
{
    if (!config().blackboard) {
        return;
    }

    config().blackboard->set("last_rune_activation_success", success);
    config().blackboard->set("last_rune_activation_result", result);
    config().blackboard->set(
        "last_rune_type",
        std::string(rune_type_ == RuneType::SMALL ? "small" : "large"));
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
    scan_mode_disabled_ = false;
    auto_shoot_enabled_ = false;
    setRuneOutcome(false, "running");
    if (config().blackboard) {
        config().blackboard->set("in_rune_phase", 1);
    }

    RCLCPP_INFO(
        node_->get_logger(),
        "EngageRune: 进入%s流程，等待按顺序发送 scan_mode=false -> yaw_controller=true -> autoshoot=true -> 激活请求",
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
        setRuneOutcome(false, "timeout");
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

    if (saw_activating_state_ && rune_status == 1) {
        RCLCPP_INFO(
            node_->get_logger(),
            "EngageRune: %s已确认激活成功，恢复 scan mode 并关闭 autoshoot",
            runeTypeName());
        setRuneOutcome(true, "activated");
        cleanupOutputs();
        return BT::NodeStatus::SUCCESS;
    }

    if (saw_activating_state_ && rune_status == 0) {
        RCLCPP_WARN(
            node_->get_logger(),
            "EngageRune: %s激活窗口结束但未激活成功，恢复 scan mode 并关闭 autoshoot",
            runeTypeName());
        setRuneOutcome(false, "window_expired");
        cleanupOutputs();
        return BT::NodeStatus::FAILURE;
    }

    if (rune_status == 1) {
        RCLCPP_INFO(
            node_->get_logger(),
            "EngageRune: %s已处于已激活状态，无需继续请求激活",
            runeTypeName());
        setRuneOutcome(true, "already_activated");
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

    if (!scan_mode_disabled_ && publishScanMode(false)) {
        scan_mode_disabled_ = true;
        RCLCPP_INFO(node_->get_logger(), "EngageRune: 已按顺序向 scan mode 发送 false");
    }

    if (triggerYawController()) {
        RCLCPP_INFO(node_->get_logger(), "EngageRune: 已按顺序向 /yaw_controller 发送 true");
    }

    if (!auto_shoot_enabled_ && publishAutoShoot(true)) {
        auto_shoot_enabled_ = true;
        RCLCPP_INFO(node_->get_logger(), "EngageRune: 已按顺序向 autoshoot 发送 true");
    }

    auto packet = utils_->buildSentryCmdPacket(resolvePostureEnum(), true);
    last_request_time_ = now_tp;
    if (send_packet(packet)) {
        RCLCPP_INFO(
            node_->get_logger(),
            "EngageRune: 已按顺序最后发送 0x0120 bit23，请求%s进入正在激活状态",
            runeTypeName());
    }

    return BT::NodeStatus::RUNNING;
}

void EngageRune::onHalted()
{
    setRuneOutcome(false, "halted");
    cleanupOutputs();
}

} // namespace sentry_nav_bt_test

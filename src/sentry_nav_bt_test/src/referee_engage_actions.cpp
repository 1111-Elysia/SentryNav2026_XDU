#include "sentry_nav_bt_test/referee_actions.hpp"

#include <algorithm>
#include <chrono>

namespace sentry_nav_bt_test
{

namespace
{

constexpr auto kRuneActivationDelayAfterYaw = std::chrono::milliseconds(2000);
constexpr int kRuneRequiredProjectileAllowance = 100;

struct SentryDecisionFeedback
{
    uint16_t exchanged_ammo{0};
    uint8_t remote_projectile_exchange_count{0};
    uint8_t remote_hp_exchange_count{0};
};

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

SentryDecisionFeedback getSentryDecisionFeedback(const BT::Blackboard::Ptr &blackboard)
{
    SentryDecisionFeedback feedback;
    if (!blackboard) {
        return feedback;
    }

    int value = 0;
    if (getBlackboardIntLike(blackboard, "exchanged_ammo", value)) {
        feedback.exchanged_ammo = static_cast<uint16_t>(std::clamp(value, 0, 0x7FF));
    }
    if (getBlackboardIntLike(blackboard, "remote_projectile_exchange_count", value)) {
        feedback.remote_projectile_exchange_count =
            static_cast<uint8_t>(std::clamp(value, 0, 0x0F));
    }
    if (getBlackboardIntLike(blackboard, "remote_hp_exchange_count", value)) {
        feedback.remote_hp_exchange_count = static_cast<uint8_t>(std::clamp(value, 0, 0x0F));
    }
    return feedback;
}

} // namespace

EngageRune::EngageRune(const std::string &name, const BT::NodeConfig &config)
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
        BT::InputPort<int>("request_interval_ms", 1000, "重复发送 0x0120 bit24 的最小间隔")};
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
    return control_publishers_ && control_publishers_->publishYawController(0);
}

bool EngageRune::ensureEngageOutputs()
{
    if (!scan_mode_yaw_control_enabled_) {
        if (!publishScanMode(true)) {
            return false;
        }
        scan_mode_yaw_control_enabled_ = true;
        RCLCPP_INFO(node_->get_logger(), "EngageRune: 已向 scan mode 发送 1，切换为定向 yaw 控制");
    }

    if (!yaw_controller_triggered_) {
        if (!triggerYawController()) {
            return false;
        }
        yaw_controller_triggered_ = true;
        yaw_controller_trigger_time_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(node_->get_logger(), "EngageRune: 已按顺序向 /yaw_controller 发送 0");
    }

    if (!auto_shoot_enabled_) {
        if (!publishAutoShoot(true)) {
            return false;
        }
        auto_shoot_enabled_ = true;
        RCLCPP_INFO(node_->get_logger(), "EngageRune: 已按顺序向 autoshoot 发送 true");
    }

    return true;
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

    if (scan_mode_yaw_control_enabled_) {
        publishScanMode(false);
        scan_mode_yaw_control_enabled_ = false;
    }

    yaw_controller_triggered_ = false;
    yaw_controller_trigger_time_ = std::chrono::steady_clock::time_point{};
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
    yaw_controller_trigger_time_ = std::chrono::steady_clock::time_point{};
    saw_activating_state_ = false;
    scan_mode_yaw_control_enabled_ = false;
    yaw_controller_triggered_ = false;
    auto_shoot_enabled_ = false;
    setRuneOutcome(false, "running");
    if (config().blackboard) {
        config().blackboard->set("in_rune_phase", 1);
    }

    RCLCPP_INFO(
        node_->get_logger(),
        "EngageRune: 进入激活能量机关流程，等待按顺序发送 scan_mode=true -> yaw_controller=0 -> autoshoot=true -> yaw后延时2s -> 激活请求");

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

    int small_rune_status = 0;
    int large_rune_status = 0;
    const bool have_small_status =
        getBlackboardIntLike(config().blackboard, "small_rune_status", small_rune_status);
    const bool have_large_status =
        getBlackboardIntLike(config().blackboard, "large_rune_status", large_rune_status);
    if (!have_small_status || !have_large_status) {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            1000,
            "EngageRune: 尚未同时获取大小能量机关状态");
        return BT::NodeStatus::RUNNING;
    }

    int can_activate_rune = 0;
    const bool have_can_activate = tryGetCanActivateRune(can_activate_rune);

    const int requested_rune_status =
        rune_type_ == RuneType::SMALL ? small_rune_status : large_rune_status;

    if (requested_rune_status == 2) {
        if (!saw_activating_state_) {
            saw_activating_state_ = true;
            RCLCPP_INFO(node_->get_logger(), "EngageRune: %s进入正在激活状态", runeTypeName());
        }
        ensureEngageOutputs();
        return BT::NodeStatus::RUNNING;
    }

    if (saw_activating_state_ && requested_rune_status == 1) {
        RCLCPP_INFO(
            node_->get_logger(),
            "EngageRune: %s已确认激活成功，恢复 scan mode 并关闭 autoshoot",
            runeTypeName());
        setRuneOutcome(true, "activated");
        cleanupOutputs();
        return BT::NodeStatus::SUCCESS;
    }

    if (saw_activating_state_ && requested_rune_status == 0) {
        RCLCPP_WARN(
            node_->get_logger(),
            "EngageRune: %s激活窗口结束但未激活成功，恢复 scan mode 并关闭 autoshoot",
            runeTypeName());
        setRuneOutcome(false, "window_expired");
        cleanupOutputs();
        return BT::NodeStatus::FAILURE;
    }

    if (requested_rune_status == 1) {
        RCLCPP_INFO(
            node_->get_logger(),
            "EngageRune: %s已处于已激活状态，无需继续请求激活",
            runeTypeName());
        setRuneOutcome(true, "already_activated");
        cleanupOutputs();
        return BT::NodeStatus::SUCCESS;
    }

    if (!have_can_activate || can_activate_rune != 1) {
        return BT::NodeStatus::RUNNING;
    }

    int projectile_allowance_17mm = 0;
    if (!getBlackboardIntLike(
            config().blackboard,
            "projectile_allowance_17mm",
            projectile_allowance_17mm)) {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            1000,
            "EngageRune: 尚未获取 17mm 允许发弹量，暂不发送%s激活请求",
            runeTypeName());
        return BT::NodeStatus::RUNNING;
    }
    if (projectile_allowance_17mm < kRuneRequiredProjectileAllowance) {
        RCLCPP_WARN(
            node_->get_logger(),
            "EngageRune: 当前 17mm 允许发弹量=%d，小于打符所需 %d，转入回补逻辑",
            projectile_allowance_17mm,
            kRuneRequiredProjectileAllowance);
        setRuneOutcome(false, "ammo_low");
        if (config().blackboard) {
            config().blackboard->set("uc_supply_goal_index", 0);
            config().blackboard->set("uc_supply_active", 1);
        }
        cleanupOutputs();
        return BT::NodeStatus::RUNNING;
    }

    const bool interval_elapsed =
        last_request_time_.time_since_epoch().count() == 0 ||
        std::chrono::duration_cast<std::chrono::milliseconds>(now_tp - last_request_time_).count() >=
            request_interval_ms_;
    if (!interval_elapsed) {
        return BT::NodeStatus::RUNNING;
    }

    if (!ensureEngageOutputs()) {
        return BT::NodeStatus::RUNNING;
    }

    const bool yaw_delay_elapsed =
        yaw_controller_trigger_time_.time_since_epoch().count() != 0 &&
        std::chrono::duration_cast<std::chrono::milliseconds>(
            now_tp - yaw_controller_trigger_time_) >= kRuneActivationDelayAfterYaw;
    if (!yaw_delay_elapsed) {
        return BT::NodeStatus::RUNNING;
    }

    const auto feedback = getSentryDecisionFeedback(config().blackboard);
    auto packet = utils_->buildSentryCmdPacket(
        resolvePostureEnum(),
        true,
        false,
        false,
        feedback.exchanged_ammo,
        feedback.remote_projectile_exchange_count,
        feedback.remote_hp_exchange_count);
    last_request_time_ = now_tp;
    if (send_packet(packet)) {
        RCLCPP_INFO(
            node_->get_logger(),
            "EngageRune: 已按顺序最后发送 0x0120 bit24，请求%s进入正在激活状态",
            runeTypeName());
    }

    return BT::NodeStatus::RUNNING;
}

void EngageRune::onHalted()
{
    setRuneOutcome(false, "halted");
    cleanupOutputs();
}

EngageOutpost::EngageOutpost(const std::string &name, const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config)
{
    if (!config.blackboard->get("node", node_)) {
        throw std::runtime_error("Missing 'node' in blackboard");
    }

    client_ = node_->create_client<rm_referee_msgs::srv::Tx>("/rm_referee/tx");
    control_publishers_ = std::make_shared<ControlTopicPublishers>(node_);
}

bool EngageOutpost::initUtils()
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

bool EngageOutpost::send_packet(
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
            "Tx.srv 未就绪，无法发送前哨站强化姿态请求");
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

BT::PortsList EngageOutpost::providedPorts()
{
    return {
        BT::InputPort<int>("timeout_ms", 45000, "整段打前哨站流程的超时时间")};
}

bool EngageOutpost::publishScanMode(bool enabled)
{
    return control_publishers_ && control_publishers_->publishScanMode(enabled);
}

bool EngageOutpost::triggerYawController()
{
    return control_publishers_ && control_publishers_->publishYawController(1);
}

bool EngageOutpost::publishOutpostMode(bool enabled)
{
    return control_publishers_ && control_publishers_->publishOutpostMode(enabled);
}

bool EngageOutpost::requestOutpostAttackPosture()
{
    if (outpost_attack_posture_requested_) {
        return true;
    }

    if (!initUtils()) {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            1000,
            "EngageOutpost: robot_id 尚未就绪，暂不能请求强化进攻姿态");
        return false;
    }

    bool sentry_info_received = false;
    int enhanced_remaining_s = -1;
    const bool enhanced_attack_available =
        !config().blackboard ||
        !config().blackboard->get("sentry_info_received", sentry_info_received) ||
        !sentry_info_received ||
        !getBlackboardIntLike(
            config().blackboard,
            "enhanced_attack_posture_remaining_s",
            enhanced_remaining_s) ||
        enhanced_remaining_s > 0;
    const auto target_posture = enhanced_attack_available
        ? rm_protocol::SentryPosture::ENHANCED_ATTACK
        : rm_protocol::SentryPosture::ATTACK;

    const auto feedback = getSentryDecisionFeedback(config().blackboard);
    auto packet = utils_->buildSentryCmdPacket(
        target_posture,
        false,
        false,
        false,
        feedback.exchanged_ammo,
        feedback.remote_projectile_exchange_count,
        feedback.remote_hp_exchange_count);

    if (!send_packet(packet)) {
        return false;
    }

    outpost_attack_posture_requested_ = true;
    if (enhanced_attack_available) {
        RCLCPP_INFO(node_->get_logger(), "EngageOutpost: 已请求强化进攻姿态");
    } else {
        RCLCPP_INFO(
            node_->get_logger(),
            "EngageOutpost: 强化进攻姿态累计时长已耗尽，改为请求普通进攻姿态");
    }
    return true;
}

bool EngageOutpost::ensureEngageOutputs()
{
    if (!requestOutpostAttackPosture()) {
        return false;
    }

    if (!scan_mode_yaw_control_enabled_) {
        if (!publishScanMode(true)) {
            return false;
        }
        scan_mode_yaw_control_enabled_ = true;
        RCLCPP_INFO(node_->get_logger(), "EngageOutpost: 已向 scan mode 发送 1，切换为定向 yaw 控制");
    }

    if (!yaw_controller_triggered_) {
        if (!triggerYawController()) {
            return false;
        }
        yaw_controller_triggered_ = true;
        RCLCPP_INFO(node_->get_logger(), "EngageOutpost: 已按顺序向 /yaw_controller 发送 1");
    }

    if (!outpost_mode_enabled_) {
        if (!publishOutpostMode(true)) {
            return false;
        }
        outpost_mode_enabled_ = true;
        RCLCPP_INFO(node_->get_logger(), "EngageOutpost: 已按顺序向 outpost_mode_type 发送 true");
    }

    return true;
}

bool EngageOutpost::isEnemyOutpostDestroyed() const
{
    int enemy_outpost_hp = 0;
    if (!getBlackboardIntLike(config().blackboard, "enemy_outpost_hp", enemy_outpost_hp)) {
        return false;
    }
    return enemy_outpost_hp <= 0;
}

void EngageOutpost::cleanupOutputs()
{
    if (outpost_mode_enabled_) {
        publishOutpostMode(false);
        outpost_mode_enabled_ = false;
    }

    if (scan_mode_yaw_control_enabled_) {
        publishScanMode(false);
        scan_mode_yaw_control_enabled_ = false;
    }

    yaw_controller_triggered_ = false;
}

void EngageOutpost::setOutpostOutcome(bool success, const std::string &result) const
{
    if (!config().blackboard) {
        return;
    }

    config().blackboard->set("last_outpost_engage_success", success);
    config().blackboard->set("last_outpost_engage_result", result);
}

BT::NodeStatus EngageOutpost::onStart()
{
    getInput("timeout_ms", timeout_ms_);
    if (timeout_ms_ < 1) {
        timeout_ms_ = 1;
    }

    start_time_ = std::chrono::steady_clock::now();
    scan_mode_yaw_control_enabled_ = false;
    yaw_controller_triggered_ = false;
    outpost_mode_enabled_ = false;
    outpost_attack_posture_requested_ = false;
    setOutpostOutcome(false, "running");

    RCLCPP_INFO(
        node_->get_logger(),
        "EngageOutpost: 进入打前哨站流程，等待按顺序发送 scan_mode=true -> yaw_controller=1 -> outpost_mode_type=true");

    if (isEnemyOutpostDestroyed()) {
        setOutpostOutcome(true, "enemy_outpost_destroyed_by_hp");
        RCLCPP_INFO(node_->get_logger(), "EngageOutpost: enemy_outpost_hp<=0，对方前哨站已被击毁");
        return BT::NodeStatus::SUCCESS;
    }

    if (!ensureEngageOutputs()) {
        cleanupOutputs();
        return BT::NodeStatus::RUNNING;
    }

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus EngageOutpost::onRunning()
{
    rclcpp::spin_some(node_);

    const auto now_tp = std::chrono::steady_clock::now();
    const auto elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now_tp - start_time_).count();

    if (isEnemyOutpostDestroyed()) {
        setOutpostOutcome(true, "enemy_outpost_destroyed_by_hp");
        cleanupOutputs();
        RCLCPP_INFO(node_->get_logger(), "EngageOutpost: enemy_outpost_hp<=0，对方前哨站已被击毁");
        return BT::NodeStatus::SUCCESS;
    }

    if (elapsed_ms > timeout_ms_) {
        RCLCPP_WARN(node_->get_logger(), "EngageOutpost: 打前哨站流程超时");
        setOutpostOutcome(false, "timeout");
        cleanupOutputs();
        return BT::NodeStatus::FAILURE;
    }

    if (!ensureEngageOutputs()) {
        cleanupOutputs();
        return BT::NodeStatus::RUNNING;
    }

    return BT::NodeStatus::RUNNING;
}

void EngageOutpost::onHalted()
{
    setOutpostOutcome(false, "halted");
    cleanupOutputs();
}

} // namespace sentry_nav_bt_test

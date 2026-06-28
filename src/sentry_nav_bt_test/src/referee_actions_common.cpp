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

RefereeActionBase::RefereeActionBase(const std::string &name, const BT::NodeConfig &config)
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

MaintainSentryPosture::MaintainSentryPosture(const std::string &name, const BT::NodeConfig &config)
    : RefereeActionBase(name, config)
{
}

BT::PortsList MaintainSentryPosture::providedPorts()
{
    return {
        BT::InputPort<int>("mode", "1:Attack, 2:Defend, 3:Move, 4:EnhancedAttack, 5:EnhancedDefend, 6:EnhancedMove"),
        BT::InputPort<int>("cooldown_ms", 5000, "全局姿态切换冷却时间(ms)，冷却内不重复发同类请求"),
        BT::InputPort<bool>("force", false, "忽略姿态切换冷却，立即发送本次姿态请求")};
}

BT::NodeStatus MaintainSentryPosture::tick()
{
    int target_mode_int;
    int cooldown_ms;
    bool force = false;
    if (!getInput("mode", target_mode_int)) {
        return BT::NodeStatus::FAILURE;
    }
    getInput("cooldown_ms", cooldown_ms);
    getInput("force", force);

    if (config().blackboard) {
        if (target_mode_int >= 4 && target_mode_int <= 6) {
            bool sentry_info_received = false;
            const char *remaining_keys[] = {
                "enhanced_attack_posture_remaining_s",
                "enhanced_defense_posture_remaining_s",
                "enhanced_move_posture_remaining_s"};
            int remaining_s = -1;
            const char *remaining_key = remaining_keys[target_mode_int - 4];
            if (config().blackboard->get("sentry_info_received", sentry_info_received) &&
                sentry_info_received &&
                getBlackboardIntLike(config().blackboard, remaining_key, remaining_s) &&
                remaining_s <= 0) {
                const int requested_mode = target_mode_int;
                target_mode_int -= 3;
                RCLCPP_INFO_THROTTLE(
                    node_->get_logger(),
                    *node_->get_clock(),
                    5000,
                    "MaintainSentryPosture: 强化姿态 %d 已耗尽，自动维持对应普通姿态 %d",
                    requested_mode,
                    target_mode_int);
            }
        }

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
    if (!getBlackboardIntLike(config().blackboard, "current_effective_posture", current_real_posture)) {
        getBlackboardIntLike(config().blackboard, "current_posture", current_real_posture);
    }
    if (current_real_posture == target_mode_int) {
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
        if (!config().blackboard->get(kLastPostureRequestTxOkKey, last_tx_ok)) {
            last_tx_ok = false;
        }
        if (!config().blackboard->get(kLastPostureRequestPendingKey, last_pending)) {
            last_pending = false;
        }
        getBlackboardDoubleLike(config().blackboard, kLastPostureRequestTimeKey, last_request_time_s);
    }

    if (last_pending &&
        current_real_posture >= 1 && current_real_posture <= 6 &&
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
    const bool force_bypasses_cooldown = force && last_target_mode != target_mode_int;
    if (cooldown_active && !force_bypasses_cooldown) {
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
    const auto feedback = getSentryDecisionFeedback(config().blackboard);
    auto packet = utils_->buildSentryCmdPacket(
        posture_enum,
        false,
        false,
        false,
        feedback.exchanged_ammo,
        feedback.remote_projectile_exchange_count,
        feedback.remote_hp_exchange_count);
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

ResolveSentryPosture::ResolveSentryPosture(
    const std::string &name,
    const BT::NodeConfig &config)
    : BT::SyncActionNode(name, config)
{
}

BT::PortsList ResolveSentryPosture::providedPorts()
{
    return {
        BT::InputPort<int>("requested_mode", "请求姿态，1-3 为普通姿态，4-6 为强化姿态"),
        BT::OutputPort<int>("resolved_mode", "强化窗口耗尽后回退得到的实际目标姿态")};
}

BT::NodeStatus ResolveSentryPosture::tick()
{
    int requested_mode = 0;
    if (!getInput("requested_mode", requested_mode) ||
        requested_mode < 1 || requested_mode > 6) {
        RCLCPP_ERROR(
            rclcpp::get_logger("ResolveSentryPosture"),
            "请求姿态必须在 1-6 范围内，当前值: %d",
            requested_mode);
        return BT::NodeStatus::FAILURE;
    }

    int resolved_mode = requested_mode;
    if (requested_mode >= 4 && config().blackboard) {
        bool sentry_info_received = false;
        const char *remaining_key = nullptr;
        switch (requested_mode) {
            case 4:
                remaining_key = "enhanced_attack_posture_remaining_s";
                break;
            case 5:
                remaining_key = "enhanced_defense_posture_remaining_s";
                break;
            case 6:
                remaining_key = "enhanced_move_posture_remaining_s";
                break;
            default:
                break;
        }

        int remaining_s = -1;
        if (config().blackboard->get("sentry_info_received", sentry_info_received) &&
            sentry_info_received && remaining_key &&
            getBlackboardIntLike(config().blackboard, remaining_key, remaining_s) &&
            remaining_s <= 0) {
            resolved_mode = requested_mode - 3;
        }
    }

    setOutput("resolved_mode", resolved_mode);
    if (requested_mode != last_requested_mode_ || resolved_mode != last_resolved_mode_) {
        if (resolved_mode != requested_mode) {
            RCLCPP_INFO(
                rclcpp::get_logger("ResolveSentryPosture"),
                "强化姿态 %d 的累计时长已耗尽，目标回退为普通姿态 %d",
                requested_mode,
                resolved_mode);
        }
        last_requested_mode_ = requested_mode;
        last_resolved_mode_ = resolved_mode;
    }

    return BT::NodeStatus::SUCCESS;
}

ConfirmResurrection::ConfirmResurrection(const std::string &name, const BT::NodeConfig &config)
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
    const auto feedback = getSentryDecisionFeedback(config().blackboard);
    auto packet = utils_->buildSentryCmdPacket(
        posture_enum,
        false,
        true,
        false,
        feedback.exchanged_ammo,
        feedback.remote_projectile_exchange_count,
        feedback.remote_hp_exchange_count);

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

BuySentryProjectile::BuySentryProjectile(const std::string &name, const BT::NodeConfig &config)
    : RefereeActionBase(name, config)
{
}

BT::PortsList BuySentryProjectile::providedPorts()
{
    return {
        BT::InputPort<int>("target_allowance", 150, "回补完成所需的 17mm 允许发弹量"),
        BT::InputPort<int>("max_exchange_projectile", 300, "哨兵补血点补弹累计兑换上限"),
        BT::InputPort<int>("reserve_gold", 300, "买弹后需要大于等于该值的剩余金币"),
        BT::InputPort<int>("buy_step", 10, "补血点买弹粒度"),
        BT::InputPort<int>("min_interval_ms", 30000, "两次买弹请求的最小间隔"),
        BT::InputPort<int>("posture", 0, "买弹请求附带姿态 (0 表示自动读取当前姿态)"),
        BT::InputPort<int>("response_timeout_ms", 200, "Tx.srv 响应超时")};
}

int BuySentryProjectile::resolveRequestedPosture(int requested_posture) const
{
    if (requested_posture >= 1 && requested_posture <= 3) {
        return requested_posture;
    }

    int current_val = 3;
    if (getBlackboardIntLike(config().blackboard, "current_posture", current_val) &&
        current_val >= 1 && current_val <= 3) {
        return current_val;
    }
    return 3;
}

void BuySentryProjectile::setBuyStatus(
    const std::string &result,
    int buy_amount,
    int exchange_target,
    bool tx_ok) const
{
    if (!config().blackboard) {
        return;
    }

    config().blackboard->set("last_sentry_projectile_buy_result", result);
    config().blackboard->set("last_sentry_projectile_buy_amount", buy_amount);
    config().blackboard->set("last_sentry_projectile_exchange_target", exchange_target);
    config().blackboard->set("last_sentry_projectile_buy_tx_ok", tx_ok);
    config().blackboard->set("last_sentry_projectile_buy_time_s", steadyNowSeconds());
}

BT::NodeStatus BuySentryProjectile::tick()
{
    int target_allowance = 150;
    int max_exchange_projectile = 300;
    int reserve_gold = 300;
    int buy_step = 10;
    int min_interval_ms = 30000;
    int requested_posture = 0;
    int response_timeout_ms = 200;

    getInput("target_allowance", target_allowance);
    getInput("max_exchange_projectile", max_exchange_projectile);
    getInput("reserve_gold", reserve_gold);
    getInput("buy_step", buy_step);
    getInput("min_interval_ms", min_interval_ms);
    getInput("posture", requested_posture);
    getInput("response_timeout_ms", response_timeout_ms);

    target_allowance = std::max(target_allowance, 0);
    max_exchange_projectile = std::max(max_exchange_projectile, 0);
    reserve_gold = std::max(reserve_gold, 0);
    buy_step = std::max(buy_step, 1);
    min_interval_ms = std::max(min_interval_ms, 0);
    response_timeout_ms = std::max(response_timeout_ms, 1);

    int current_allowance = 0;
    if (!getBlackboardIntLike(config().blackboard, "projectile_allowance_17mm", current_allowance)) {
        setBuyStatus("allowance_unavailable", 0, 0, false);
        return BT::NodeStatus::FAILURE;
    }

    if (current_allowance >= target_allowance) {
        setBuyStatus("already_enough", 0, 0, true);
        return BT::NodeStatus::SUCCESS;
    }

    bool sentry_info_received = false;
    if (!config().blackboard->get("sentry_info_received", sentry_info_received) ||
        !sentry_info_received) {
        setBuyStatus("sentry_info_unavailable", 0, 0, false);
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            1000,
            "BuySentryProjectile: 尚未收到 0x020D sentry_info，暂不发送买弹请求");
        return BT::NodeStatus::FAILURE;
    }

    int remaining_gold_coin = 0;
    int exchanged_ammo = 0;
    if (!getBlackboardIntLike(config().blackboard, "remaining_gold_coin", remaining_gold_coin) ||
        !getBlackboardIntLike(config().blackboard, "exchanged_ammo", exchanged_ammo)) {
        setBuyStatus("gold_or_exchange_unavailable", 0, 0, false);
        return BT::NodeStatus::FAILURE;
    }

    if (!initUtils()) {
        setBuyStatus("robot_id_unavailable", 0, 0, false);
        return BT::NodeStatus::FAILURE;
    }

    const auto now_tp = std::chrono::steady_clock::now();
    if (last_send_time_.time_since_epoch().count() != 0 &&
        std::chrono::duration_cast<std::chrono::milliseconds>(now_tp - last_send_time_).count() <
            min_interval_ms) {
        setBuyStatus("cooldown", 0, 0, true);
        return BT::NodeStatus::SUCCESS;
    }

    const int amount_to_target = std::max(0, target_allowance - current_allowance);
    const int amount_needed_by_step =
        ((amount_to_target + buy_step - 1) / buy_step) * buy_step;
    const int exchange_capacity =
        std::max(0, max_exchange_projectile - exchanged_ammo);
    const int spendable_gold =
        std::max(0, remaining_gold_coin - reserve_gold);
    const int affordable_by_step = (spendable_gold / buy_step) * buy_step;
    const int buy_amount = std::min({amount_needed_by_step, exchange_capacity, affordable_by_step});

    if (buy_amount < buy_step) {
        const char *reason = "cannot_buy";
        if (exchange_capacity < buy_step) {
            reason = "exchange_limit_reached";
        } else if (affordable_by_step < buy_step) {
            reason = "gold_reserve_protected";
        }
        setBuyStatus(reason, 0, exchanged_ammo, false);
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            2000,
            "BuySentryProjectile: 当前允许发弹量=%d，目标=%d，金币=%d，累计已兑=%d，暂不能买弹(%s)",
            current_allowance,
            target_allowance,
            remaining_gold_coin,
            exchanged_ammo,
            reason);
        return BT::NodeStatus::SUCCESS;
    }

    const int exchange_target = exchanged_ammo + buy_amount;
    const int posture_int = resolveRequestedPosture(requested_posture);
    auto posture_enum = static_cast<rm_protocol::SentryPosture>(posture_int);
    const auto feedback = getSentryDecisionFeedback(config().blackboard);
    auto packet = utils_->buildSentryCmdPacket(
        posture_enum,
        false,
        false,
        false,
        static_cast<uint16_t>(exchange_target),
        feedback.remote_projectile_exchange_count,
        feedback.remote_hp_exchange_count);

    const bool tx_ok = send_packet(packet, std::chrono::milliseconds(response_timeout_ms));
    if (tx_ok) {
        last_send_time_ = now_tp;
        RCLCPP_INFO(
            node_->get_logger(),
            "BuySentryProjectile: 当前允许=%d，目标=%d，本次买=%d，累计兑换目标=%d，金币=%d(保留>=%d)",
            current_allowance,
            target_allowance,
            buy_amount,
            exchange_target,
            remaining_gold_coin,
            reserve_gold);
    }

    setBuyStatus(tx_ok ? "sent" : "tx_failed", buy_amount, exchange_target, tx_ok);
    return tx_ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

} // namespace sentry_nav_bt_test

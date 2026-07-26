#include "sentry_nav_bt/topic_listener.hpp"

#include <string>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/blackboard.h"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// 裁判系统交互接口
#include "rm_referee_msgs/msg/robot_pos.hpp"
#include "rm_referee_msgs/msg/rfid_status.hpp"
#include "rm_referee_msgs/msg/sentry_info.hpp"
#include "rm_referee_msgs/msg/game_status.hpp"
#include "rm_referee_msgs/msg/robot_status.hpp"
#include "rm_referee_msgs/msg/game_robot_hp.hpp"
#include "rm_referee_msgs/msg/projectile_allowance.hpp"
#include "rm_referee_msgs/msg/power_heat_data.hpp"
#include "rm_referee_msgs/msg/hurt_data.hpp"
#include "rm_referee_msgs/msg/event_data.hpp"
#include "rm_referee_msgs/msg/map_command.hpp"
#include "rm_referee_msgs/msg/ground_robot_position.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include "sentry_nav_bt/center_hold_vw_controller.hpp"
#include "sentry_nav_bt/blackboard/defaults.hpp"
#include "sentry_nav_bt/blackboard_utils.hpp"
#include "sentry_nav_bt/navigation/waypoint_loader.hpp"
#include "sentry_nav_bt/trapezoid_highland.hpp"

namespace sentry_nav_bt
{
    /**
     * @brief 黑板管理器类，简化自定义话题的订阅和黑板更新
     */
    class BlackboardManager::Impl
    {
    public:
        /**
         * @brief 构造函数
         * @param node ROS节点指针
         * @param blackboard 行为树黑板指针
         */
        Impl(rclcpp::Node::SharedPtr node, BT::Blackboard::Ptr blackboard)
            : node_(node), blackboard_(blackboard)
        {
            RCLCPP_INFO(node_->get_logger(), "初始化黑板管理器");
        }

        template <typename T>
        void subscribeWithProcessorBestEffort(
            const std::string &topic_name,
            std::function<void(const typename T::SharedPtr, BT::Blackboard::Ptr)> processor)
        {
            auto qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
            qos.best_effort();         // 设置为BEST_EFFORT可靠性
            qos.durability_volatile(); // 设置为VOLATILE持久性

            auto callback = [this, processor](const typename T::SharedPtr msg)
            {
                processor(msg, blackboard_);
            };

            auto subscription = node_->create_subscription<T>(
                topic_name, qos, callback);

            subscriptions_.push_back(subscription);
        }

        // 订阅裁判系统话题
        void bb_manager_init()
        {
            blackboard::initializeDefaults(blackboard_);

            // 初始化定时器
            hurt_reset_timer_ = node_->create_wall_timer(
                std::chrono::seconds(5),
                [this]()
                {
                    std::lock_guard<std::mutex> lock(hurt_mutex_);
                    
                    int attack_status = 0;
                    // 读取当前状态，如果是 1 则打印日志
                    if (blackboard_->get("is_under_attack", attack_status) && attack_status == 1)
                    {
                        RCLCPP_INFO(node_->get_logger(), "攻击停止 (5s超时)，受击状态复位");
                    }

                    blackboard_->set("is_under_attack", 0); 
                    blackboard_->set("hurt_armor_id", -1); 
                    blackboard_->set("hurt_armor", 0);
                });

            hero_position_stale_timer_ = node_->create_wall_timer(
                std::chrono::milliseconds(250),
                [this]()
                {
                    std::lock_guard<std::mutex> lock(hero_position_mutex_);
                    if (!hero_position_received_) {
                        return;
                    }

                    const double now_s = steadyNowSeconds();
                    if (now_s - hero_position_last_update_s_ <= kHeroPositionStaleTimeoutSeconds) {
                        return;
                    }

                    hero_highland_inside_ = false;
                    hero_highland_elapsed_s_ = 0.0;
                    blackboard_->set("hero_position_fresh", 0);
                    blackboard_->set("hero_highland_inside", 0);
                    blackboard_->set("hero_highland_elapsed_ms", 0);
                });

            map_command_window_timer_ = node_->create_wall_timer(
                std::chrono::milliseconds(100),
                [this]()
                {
                    std::lock_guard<std::mutex> lock(map_command_mutex_);
                    const int current_window_id =
                        competitionTimeWindowId(getBlackboardDouble("stage_remain_time", 0.0));

                    if (map_command_observed_window_id_ < 0) {
                        map_command_observed_window_id_ = current_window_id;
                        return;
                    }
                    if (current_window_id == map_command_observed_window_id_) {
                        return;
                    }

                    bool had_map_command = false;
                    if (!blackboard_->get("map_command_received", had_map_command)) {
                        had_map_command = false;
                    }
                    map_command_observed_window_id_ = current_window_id;
                    blackboard_->set("map_command_received", false);
                    blackboard_->set("map_command_window_id", -1);
                    blackboard_->set("uc_highland_hold_active", 0);

                    if (had_map_command) {
                        RCLCPP_INFO(
                            node_->get_logger(),
                            "比赛时间窗切换至 Phase %d，清除上一时间窗的0x0303触发状态",
                            current_window_id);
                    }
                });

            // 订阅己方机器人血量数据
            this->subscribeWithProcessorBestEffort<rm_referee_msgs::msg::GameRobotHP>(
                "/rm_referee/game_robot_hp",
                [this](const rm_referee_msgs::msg::GameRobotHP::SharedPtr msg, BT::Blackboard::Ptr bb)
                {
                    bb->set("ally_1_robot_hp", msg->ally_1_robot_hp);
                    bb->set("ally_2_robot_hp", msg->ally_2_robot_hp);
                    bb->set("ally_3_robot_hp", msg->ally_3_robot_hp);
                    bb->set("ally_4_robot_hp", msg->ally_4_robot_hp);
                    bb->set("damage_difference", msg->damage_difference);
                    bb->set("ally_7_robot_hp", msg->ally_7_robot_hp);
                    bb->set("ally_outpost_hp", msg->ally_outpost_hp); // 前哨站
                    bb->set("ally_base_hp", msg->ally_base_hp);       // 基地
                    bb->set("enemy_outpost_hp", msg->enemy_outpost_hp);
                    bb->set("enemy_base_hp", msg->enemy_base_hp);
                    RCLCPP_DEBUG(
                        node_->get_logger(),
                        "黑板更新: '%s' 从话题 '%s'",
                        "game_robot_hp",
                        "/rm_referee/game_robot_hp");
                });

            // 订阅场地事件数据（0x0101)
            this->subscribeWithProcessorBestEffort<rm_referee_msgs::msg::EventData>(
                "/rm_referee/event_data",
                [this](const rm_referee_msgs::msg::EventData::SharedPtr msg, BT::Blackboard::Ptr bb)
                {
                    // 0:未激活, 1:已激活
                    uint32_t event_data = msg->event_data;
                    // 解析大能量机关状态 (Bit 5-6)
                    uint8_t large_rune_status = (event_data >> 5) & 0x03;
                    // 解析小能量机关状态 (Bit 3-4)
                    uint8_t small_rune_status = (event_data >> 3) & 0x03;
                    // 解析中心增益点占领状态 (Bit 23-24, 仅 RMUL 适用)
                    uint8_t center_gain_point_occupancy_status = (event_data >> 23) & 0x03;
                    // 解析堡垒/前哨站/基地增益点占领状态 (Bit 25-29)
                    uint8_t fortress_gain_point_occupancy_status = (event_data >> 25) & 0x03;
                    uint8_t outpost_gain_point_occupancy_status = (event_data >> 27) & 0x03;
                    uint8_t base_gain_point_occupied = (event_data >> 29) & 0x01;

                    bb->set("large_rune_status", large_rune_status);
                    bb->set("small_rune_status", small_rune_status);
                    bb->set(
                        "center_gain_point_occupancy_status",
                        static_cast<int>(center_gain_point_occupancy_status));
                    bb->set(
                        "fortress_gain_point_occupancy_status",
                        static_cast<int>(fortress_gain_point_occupancy_status));
                    bb->set(
                        "outpost_gain_point_occupancy_status",
                        static_cast<int>(outpost_gain_point_occupancy_status));
                    bb->set("base_gain_point_occupied", static_cast<int>(base_gain_point_occupied));
                    RCLCPP_DEBUG(
                        node_->get_logger(),
                        "黑板更新: 大符状态=%d, 小符状态=%d, 中心增益点=%d, 堡垒增益点=%d, 前哨增益点=%d, 基地增益点=%d",
                        large_rune_status,
                        small_rune_status,
                        center_gain_point_occupancy_status,
                        fortress_gain_point_occupancy_status,
                        outpost_gain_point_occupancy_status,
                        base_gain_point_occupied);
                });

            // 订阅机器人位置数据
            this->subscribeWithProcessorBestEffort<rm_referee_msgs::msg::RobotPos>(
                "/rm_referee/robot_pos",
                [this](const rm_referee_msgs::msg::RobotPos::SharedPtr msg, BT::Blackboard::Ptr bb)
                {
                    bb->set("robot_x", msg->x);
                    bb->set("robot_y", msg->y);
                    bb->set("robot_angle", msg->angle);
                    RCLCPP_DEBUG(
                        node_->get_logger(),
                        "黑板更新: '%s' 从话题 '%s'",
                        "robot_pos",
                        "/rm_referee/robot_pos");
                });

            // 订阅己方地面机器人位置（0x020B），持续计算英雄在梯形高地内的停留时间。
            this->subscribeWithProcessorBestEffort<rm_referee_msgs::msg::GroundRobotPosition>(
                "/rm_referee/ground_robot_position",
                [this](const rm_referee_msgs::msg::GroundRobotPosition::SharedPtr msg, BT::Blackboard::Ptr bb)
                {
                    std::lock_guard<std::mutex> lock(hero_position_mutex_);
                    const double now_s = steadyNowSeconds();

                    if (hero_position_received_ &&
                        now_s - hero_position_last_update_s_ > kHeroPositionStaleTimeoutSeconds)
                    {
                        hero_highland_inside_ = false;
                        hero_highland_elapsed_s_ = 0.0;
                    }

                    double robot_id_value = 0.0;
                    blackboard_utils::getValue(bb, "robot_id", robot_id_value, "BlackboardManager");
                    const int robot_id = static_cast<int>(robot_id_value);
                    const bool inside = trapezoid_highland::contains(
                        static_cast<double>(msg->hero_x),
                        static_cast<double>(msg->hero_y),
                        robot_id);

                    if (inside) {
                        if (!hero_highland_inside_) {
                            hero_highland_enter_time_s_ = now_s;
                        }
                        hero_highland_elapsed_s_ = now_s - hero_highland_enter_time_s_;
                    } else {
                        hero_highland_elapsed_s_ = 0.0;
                    }

                    hero_position_received_ = true;
                    hero_position_last_update_s_ = now_s;
                    hero_highland_inside_ = inside;

                    bb->set("ground_robot_position_received", true);
                    bb->set("hero_x", static_cast<double>(msg->hero_x));
                    bb->set("hero_y", static_cast<double>(msg->hero_y));
                    bb->set("hero_position_fresh", 1);
                    bb->set("hero_highland_inside", inside ? 1 : 0);
                    bb->set(
                        "hero_highland_elapsed_ms",
                        static_cast<int>(hero_highland_elapsed_s_ * 1000.0));

                    RCLCPP_DEBUG(
                        node_->get_logger(),
                        "0x020B 英雄位置: (%.2f, %.2f), robot_id=%d, 梯形高地内=%d, 连续停留=%.1fs",
                        msg->hero_x,
                        msg->hero_y,
                        robot_id,
                        inside ? 1 : 0,
                        hero_highland_elapsed_s_);
                });

            // 订阅选手端小地图交互数据（0x0303）。
            // 官方协议会在一次触发后连发5帧，并继续以1Hz重发最近一包；这里只把新触发记为事件。
            this->subscribeWithProcessorBestEffort<rm_referee_msgs::msg::MapCommand>(
                "/rm_referee/map_command",
                [this](const rm_referee_msgs::msg::MapCommand::SharedPtr msg, BT::Blackboard::Ptr bb)
                {
                    std::lock_guard<std::mutex> lock(map_command_mutex_);
                    const double now_s = steadyNowSeconds();

                    bb->set("map_command", *msg);
                    if (!isNewMapCommandEvent(*msg, now_s)) {
                        RCLCPP_DEBUG(
                            node_->get_logger(),
                            "忽略0x0303协议重复帧: x=%.2f, y=%.2f, key=%u, target_robot_id=%u, cmd_source=%u",
                            msg->target_position_x,
                            msg->target_position_y,
                            static_cast<unsigned>(msg->cmd_keyboard),
                            static_cast<unsigned>(msg->target_robot_id),
                            static_cast<unsigned>(msg->cmd_source));
                        return;
                    }

                    const int window_id =
                        competitionTimeWindowId(getBlackboardDouble("stage_remain_time", 0.0));
                    map_command_observed_window_id_ = window_id;
                    bb->set("map_command_received", true);
                    bb->set("map_command_window_id", window_id);
                    bb->set("map_command_last_event_time_s", now_s);

                    RCLCPP_INFO(
                        node_->get_logger(),
                        "收到新的0x0303小地图指令: Phase=%d, x=%.2f, y=%.2f, key=%u, target_robot_id=%u, cmd_source=%u",
                        window_id,
                        msg->target_position_x,
                        msg->target_position_y,
                        static_cast<unsigned>(msg->cmd_keyboard),
                        static_cast<unsigned>(msg->target_robot_id),
                        static_cast<unsigned>(msg->cmd_source));
                });

            // 订阅哨兵信息（0x020D）
            this->subscribeWithProcessorBestEffort<rm_referee_msgs::msg::SentryInfo>(
                "/rm_referee/sentry_info",
                [this](const rm_referee_msgs::msg::SentryInfo::SharedPtr msg, BT::Blackboard::Ptr bb)
                {
                    // 解析 sentry_info (32位)   暂时不用
                    uint32_t raw_info = msg->sentry_info;
                    // Bit 0-10: 成功兑换的发弹量
                    uint16_t exchanged_ammo = raw_info & 0x7FF;
                    // Bit 11-14 15-18 成功远程兑换的允许发弹量次数和远程兑换血量次数（暂时不用）
                    uint8_t remote_projectile_exchange_count = (raw_info >> 11) & 0x0F;
                    uint8_t remote_hp_exchange_count = (raw_info >> 15) & 0x0F;
                    // Bit 19: 是否可以确认免费复活 (1=是)
                    bool can_confirm_resurrection = (raw_info >> 19) & 0x01;
                    // Bit 20: 是否可以兑换立即复活 (1=是)
                    bool can_buy_resurrection = (raw_info >> 20) & 0x01;
                    // Bit 21-30: 立即复活所需金币数
                    uint16_t buy_resurrection_cost = (raw_info >> 21) & 0x3FF;

                    // 解析 sentry_info_2 (16位)
                    uint16_t raw_info_2 = msg->sentry_info_2;
                    // Bit 12-13: 哨兵当前姿态 (1=进攻, 2=防御, 3=移动)
                    uint8_t current_posture = (raw_info_2 >> 12) & 0x03;
                    // Bit 14: 己方能量机关是否能够进入正在激活状态 (1=可激活)  需要先确认这个位是 1，才能发指令去激活
                    bool can_activate_rune = (raw_info_2 >> 14) & 0x01;
                    // Bit 15: 当前姿态是否为强化姿态。
                    bool current_posture_enhanced = (raw_info_2 >> 15) & 0x01;
                    int current_effective_posture = static_cast<int>(current_posture);
                    if (current_posture_enhanced && current_posture >= 1 && current_posture <= 3) {
                        current_effective_posture += 3;
                    }

                    uint64_t raw_info_3 = msg->sentry_info_3;
                    uint8_t attack_posture_remaining_s = raw_info_3 & 0xFF;
                    uint8_t defense_posture_remaining_s = (raw_info_3 >> 8) & 0xFF;
                    uint8_t move_posture_remaining_s = (raw_info_3 >> 16) & 0xFF;
                    uint8_t enhanced_attack_posture_remaining_s = (raw_info_3 >> 32) & 0xFF;
                    uint8_t enhanced_defense_posture_remaining_s = (raw_info_3 >> 40) & 0xFF;
                    uint8_t enhanced_move_posture_remaining_s = (raw_info_3 >> 48) & 0xFF;

                    // 写入黑板
                    bb->set("exchanged_ammo", exchanged_ammo);
                    bb->set("remote_projectile_exchange_count", remote_projectile_exchange_count);
                    bb->set("remote_hp_exchange_count", remote_hp_exchange_count);
                    bb->set("can_confirm_resurrection", can_confirm_resurrection);
                    bb->set("can_buy_resurrection", can_buy_resurrection);
                    bb->set("buy_resurrection_cost", buy_resurrection_cost);

                    bb->set("current_posture", static_cast<int>(current_posture));
                    bb->set("current_posture_enhanced", current_posture_enhanced);
                    bb->set("current_effective_posture", current_effective_posture);
                    if (current_effective_posture >= 1 && current_effective_posture <= 6) {
                        int last_observed_posture = -1;
                        if (!bb->get(
                                "last_observed_effective_posture",
                                last_observed_posture) ||
                            last_observed_posture < 1 ||
                            last_observed_posture > 6) {
                            // 首次回显只建立基线，开局默认移动姿态可立即切换。
                            bb->set(
                                "last_observed_effective_posture",
                                current_effective_posture);
                        } else if (last_observed_posture != current_effective_posture) {
                            const double now_s =
                                std::chrono::duration<double>(
                                    std::chrono::steady_clock::now().time_since_epoch())
                                    .count();
                            bb->set(
                                "last_observed_effective_posture",
                                current_effective_posture);
                            bb->set("last_posture_change_time_s", now_s);
                        }
                    }
                    bb->set("can_activate_rune", can_activate_rune);
                    bb->set("sentry_info_3", raw_info_3);
                    bb->set("attack_posture_remaining_s", static_cast<int>(attack_posture_remaining_s));
                    bb->set("defense_posture_remaining_s", static_cast<int>(defense_posture_remaining_s));
                    bb->set("move_posture_remaining_s", static_cast<int>(move_posture_remaining_s));
                    bb->set(
                        "enhanced_attack_posture_remaining_s",
                        static_cast<int>(enhanced_attack_posture_remaining_s));
                    bb->set(
                        "enhanced_defense_posture_remaining_s",
                        static_cast<int>(enhanced_defense_posture_remaining_s));
                    bb->set(
                        "enhanced_move_posture_remaining_s",
                        static_cast<int>(enhanced_move_posture_remaining_s));
                    bb->set("sentry_info_received", true);

                    RCLCPP_DEBUG(node_->get_logger(),
                                 "哨兵信息: 姿态=%d, 强化=%d, 有效姿态=%d, 可激活符=%d, 强化进攻剩余=%d, 复活金币=%d",
                                 current_posture,
                                 current_posture_enhanced,
                                 current_effective_posture,
                                 can_activate_rune,
                                 enhanced_attack_posture_remaining_s,
                                 buy_resurrection_cost);
                });

            // 订阅游戏状态
            this->subscribeWithProcessorBestEffort<rm_referee_msgs::msg::GameStatus>(
                "/rm_referee/game_status",
                [this](const rm_referee_msgs::msg::GameStatus::SharedPtr msg, BT::Blackboard::Ptr bb)
                {
                    bb->set("game_type", msg->game_type);
                    bb->set("game_progress", msg->game_progress);
                    bb->set("stage_remain_time", msg->stage_remain_time);
                    bb->set("sync_timestamp", msg->sync_timestamp);
                    bb->set("game_status_received", true);
                    if (msg->game_progress < 4U) {
                        bb->set("is_under_attack", 0);
                        bb->set("hurt_armor_id", -1);
                        bb->set("hurt_armor", 0);
                    }
                    RCLCPP_DEBUG(
                        node_->get_logger(),
                        "黑板更新: '%s' 从话题 '%s'",
                        "game_status",
                        "/rm_referee/game_status");
                });

            // 订阅本机器人状态（哨兵）
            this->subscribeWithProcessorBestEffort<rm_referee_msgs::msg::RobotStatus>(
                "/rm_referee/robot_status",
                [this](const rm_referee_msgs::msg::RobotStatus::SharedPtr msg, BT::Blackboard::Ptr bb)
                {
                    bb->set("robot_id", msg->robot_id);
                    bb->set("robot_level", msg->robot_level);
                    bb->set("current_hp", msg->current_hp);
                    bb->set("maximum_hp", msg->maximum_hp);
                    bb->set("shooter_barrel_cooling_value", msg->shooter_barrel_cooling_value);
                    bb->set("shooter_barrel_heat_limit", msg->shooter_barrel_heat_limit);
                    bb->set("chassis_power_limit", msg->chassis_power_limit);
                    bb->set("bullet_speed_limit", msg->bullet_speed_limit);
                    bb->set("power_management_gimbal_output", msg->power_management_gimbal_output);
                    bb->set("power_management_chassis_output", msg->power_management_chassis_output);
                    bb->set("power_management_shooter_output", msg->power_management_shooter_output);
                    RCLCPP_DEBUG(
                        node_->get_logger(),
                        "黑板更新: '%s' 从话题 '%s'",
                        "robot_status",
                        "/rm_referee/robot_status");
                });

            // 订阅功率热量数据（0x0202），用于根据 17mm 热量趋势判断是否正在发射弹丸
            this->subscribeWithProcessorBestEffort<rm_referee_msgs::msg::PowerHeatData>(
                "/rm_referee/power_heat_data",
                [this](const rm_referee_msgs::msg::PowerHeatData::SharedPtr msg, BT::Blackboard::Ptr bb)
                {
                    const int current_heat = static_cast<int>(msg->shooter_17mm_1_barrel_heat);
                    const int previous_heat = has_previous_shooter_17mm_heat_
                                                  ? previous_shooter_17mm_heat_
                                                  : current_heat;
                    const int heat_delta = current_heat - previous_heat;
                    const bool heat_indicates_firing =
                        has_previous_shooter_17mm_heat_ && current_heat > 0 && heat_delta >= 0;
                    const rclcpp::Time now = node_->now();

                    if (heat_indicates_firing) {
                        shooter_17mm_heat_firing_latched_ = true;
                        shooter_17mm_heat_non_firing_since_valid_ = false;
                    } else if (shooter_17mm_heat_firing_latched_) {
                        if (!shooter_17mm_heat_non_firing_since_valid_) {
                            shooter_17mm_heat_non_firing_since_ = now;
                            shooter_17mm_heat_non_firing_since_valid_ = true;
                        }

                        const double non_firing_duration =
                            (now - shooter_17mm_heat_non_firing_since_).seconds();
                        if (non_firing_duration >= kShooter17mmDefenseDelaySeconds) {
                            shooter_17mm_heat_firing_latched_ = false;
                            shooter_17mm_heat_non_firing_since_valid_ = false;
                        }
                    }

                    const int heat_firing = shooter_17mm_heat_firing_latched_ ? 1 : 0;

                    bb->set("buffer_energy", static_cast<int>(msg->buffer_energy));
                    bb->set("shooter_17mm_barrel_heat", current_heat);
                    bb->set("shooter_17mm_barrel_heat_prev", previous_heat);
                    bb->set("shooter_17mm_barrel_heat_delta", heat_delta);
                    bb->set("shooter_17mm_heat_firing", heat_firing);
                    bb->set("shooter_42mm_barrel_heat", static_cast<int>(msg->shooter_42mm_barrel_heat));
                    bb->set("power_heat_data_received", true);

                    previous_shooter_17mm_heat_ = current_heat;
                    has_previous_shooter_17mm_heat_ = true;

                    RCLCPP_DEBUG(
                        node_->get_logger(),
                        "黑板更新: 17mm热量=%d, 上次=%d, 增量=%d, 正在射击=%d",
                        current_heat,
                        previous_heat,
                        heat_delta,
                        heat_firing);
                });

            // 订阅弹丸允许信息  0x0208
            this->subscribeWithProcessorBestEffort<rm_referee_msgs::msg::ProjectileAllowance>(
                "/rm_referee/projectile_allowance",
                [this](const rm_referee_msgs::msg::ProjectileAllowance::SharedPtr msg, BT::Blackboard::Ptr bb)
                {
                    bb->set("projectile_allowance_17mm", msg->projectile_allowance_17mm);
                    bb->set("projectile_allowance_42mm", msg->projectile_allowance_42mm);
                    bb->set("remaining_gold_coin", msg->remaining_gold_coin);
                    RCLCPP_DEBUG(
                        node_->get_logger(),
                        "黑板更新: '%s' 从话题 '%s'",
                        "projectile_allowance",
                        "/rm_referee/projectile_allowance");
                });

            // 订阅 RFID 状态（0x0209），用于读取己方堡垒增益点 RFID（bit17）
            this->subscribeWithProcessorBestEffort<rm_referee_msgs::msg::RFIDStatus>(
                "/rm_referee/rfid_status",
                [this](const rm_referee_msgs::msg::RFIDStatus::SharedPtr msg, BT::Blackboard::Ptr bb)
                {
                    const uint32_t rfid_status = msg->rfid_status;
                    const uint8_t rfid_status_2 = msg->rfid_status_2;
                    const int ally_fortress_rfid_detected = static_cast<int>((rfid_status >> 17U) & 0x01U);
                    const int supply_zone_bit19_detected = static_cast<int>((rfid_status >> 19U) & 0x01U);
                    const int supply_zone_bit20_detected = static_cast<int>((rfid_status >> 20U) & 0x01U);
                    const int supply_zone_detected =
                        (supply_zone_bit19_detected == 1 || supply_zone_bit20_detected == 1) ? 1 : 0;

                    bb->set("rfid_status", rfid_status);
                    bb->set("rfid_status_2", rfid_status_2);
                    bb->set("rfid_ally_fortress_detected", ally_fortress_rfid_detected);
                    bb->set("rfid_supply_zone_bit19_detected", supply_zone_bit19_detected);
                    bb->set("rfid_supply_zone_bit20_detected", supply_zone_bit20_detected);
                    bb->set("rfid_supply_zone_detected", supply_zone_detected);

                    RCLCPP_DEBUG(
                        node_->get_logger(),
                        "黑板更新: rfid_status=0x%08X, rfid_status_2=0x%02X, ally_fortress_rfid(bit17)=%d, supply_zone(bit19)=%d, supply_zone(bit20)=%d, supply_zone(any)=%d",
                        static_cast<unsigned int>(rfid_status),
                        static_cast<unsigned int>(rfid_status_2),
                        ally_fortress_rfid_detected,
                        supply_zone_bit19_detected,
                        supply_zone_bit20_detected,
                        supply_zone_detected);
                });

            center_hold_vw_controller_ =
                std::make_shared<CenterHoldVwController>(node_, blackboard_);
            center_hold_vw_controller_->start();

            // 订阅伤害状态数据 0x0206
            this->subscribeWithProcessorBestEffort<rm_referee_msgs::msg::HurtData>(
                "/rm_referee/hurt_data",
                [this](const rm_referee_msgs::msg::HurtData::SharedPtr msg, BT::Blackboard::Ptr bb)
                {
                    std::lock_guard<std::mutex> lock(hurt_mutex_);

                    double game_progress = 0.0;
                    if (!blackboard_utils::getValue(bb, "game_progress", game_progress, "BlackboardManager") ||
                        game_progress < 4.0) {
                        bb->set("is_under_attack", 0);
                        bb->set("hurt_armor_id", -1);
                        bb->set("hurt_armor", 0);
                        return;
                    }

                    double current_hp = 0.0;
                    if (blackboard_utils::getValue(bb, "current_hp", current_hp, "BlackboardManager") &&
                        current_hp <= 0.0) {
                        bb->set("is_under_attack", 0);
                        bb->set("hurt_armor_id", -1);
                        bb->set("hurt_armor", 0);
                        return;
                    }

                    uint8_t id = msg->armor_id;              
                    uint8_t type = msg->hp_deduction_reason; // 0=弹丸
                    // 1. 如果装甲板ID大于5，忽略 (无效数据)
                    if (id > 4) {
                        return; 
                    }  
                    // 2. 如果伤害类型不是弹丸 (type 0)，也忽略 (比如离线/撞击)
                    if (type != 0) {
                        return;
                    }
                    
                    RCLCPP_INFO(node_->get_logger(),
                                "[HurtData] 确认装甲板受击! ArmorID: %d", id);
                    bb->set("hurt_armor_id", (int)id);
                    bb->set("is_under_attack", 1); 

                    // 重置定时器
                    if (hurt_reset_timer_)
                        hurt_reset_timer_->reset();
                });

            // 初始化 TF 组件
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            // 创建定时器，每 100ms 更新一次当前位置
            tf_timer_ = node_->create_wall_timer(
                std::chrono::milliseconds(100),
                [this]()
                { updateCurrentPosition(); });

            RCLCPP_INFO(node_->get_logger(), "已启动 TF 监听，将定期更新当前位置到黑板 'waypoint_now'");
        }

        void load_waypoints(const std::string &json_file_path)
        {
            navigation::loadWaypoints(json_file_path, blackboard_, node_);
        }

    private:
        rclcpp::Node::SharedPtr node_;
        BT::Blackboard::Ptr blackboard_;
        std::vector<std::shared_ptr<rclcpp::SubscriptionBase>> subscriptions_;
        rclcpp::TimerBase::SharedPtr hurt_reset_timer_;
        std::mutex hurt_mutex_;
        rclcpp::TimerBase::SharedPtr hero_position_stale_timer_;
        std::mutex hero_position_mutex_;
        bool hero_position_received_{false};
        bool hero_highland_inside_{false};
        double hero_position_last_update_s_{0.0};
        double hero_highland_enter_time_s_{0.0};
        double hero_highland_elapsed_s_{0.0};
        static constexpr double kHeroPositionStaleTimeoutSeconds = 2.5;
        rclcpp::TimerBase::SharedPtr map_command_window_timer_;
        std::mutex map_command_mutex_;
        bool map_command_payload_seen_{false};
        rm_referee_msgs::msg::MapCommand last_map_command_payload_{};
        double map_command_last_packet_time_s_{0.0};
        bool map_command_cluster_triggered_{false};
        int map_command_observed_window_id_{-1};
        static constexpr double kMapCommandBurstGapSeconds = 0.35;

        std::shared_ptr<CenterHoldVwController> center_hold_vw_controller_;
        bool has_previous_shooter_17mm_heat_{false};
        int previous_shooter_17mm_heat_{0};
        bool shooter_17mm_heat_firing_latched_{false};
        bool shooter_17mm_heat_non_firing_since_valid_{false};
        rclcpp::Time shooter_17mm_heat_non_firing_since_{0, 0, RCL_ROS_TIME};
        static constexpr double kShooter17mmDefenseDelaySeconds = 10.0;

        // TF
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        rclcpp::TimerBase::SharedPtr tf_timer_;
        rclcpp::Time last_waypoint_update_time_{0, 0, RCL_ROS_TIME};
        bool waypoint_now_received_ = false;
        bool last_waypoint_now_valid_ = false;

        static double steadyNowSeconds()
        {
            return std::chrono::duration<double>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
        }

        static int competitionTimeWindowId(double stage_remain_time)
        {
            if (stage_remain_time > 390.0) {
                return 1;
            }
            if (stage_remain_time > 330.0) {
                return 2;
            }
            if (stage_remain_time > 300.0) {
                return 3;
            }
            if (stage_remain_time > 240.0) {
                return 4;
            }
            if (stage_remain_time > 210.0) {
                return 5;
            }
            if (stage_remain_time > 165.0) {
                return 6;
            }
            if (stage_remain_time > 135.0) {
                return 7;
            }
            if (stage_remain_time > 90.0) {
                return 8;
            }
            if (stage_remain_time > 60.0) {
                return 9;
            }
            return 10;
        }

        static bool sameMapCommandPayload(
            const rm_referee_msgs::msg::MapCommand &lhs,
            const rm_referee_msgs::msg::MapCommand &rhs)
        {
            return lhs.target_position_x == rhs.target_position_x &&
                   lhs.target_position_y == rhs.target_position_y &&
                   lhs.cmd_keyboard == rhs.cmd_keyboard &&
                   lhs.target_robot_id == rhs.target_robot_id &&
                   lhs.cmd_source == rhs.cmd_source;
        }

        bool isNewMapCommandEvent(
            const rm_referee_msgs::msg::MapCommand &msg,
            double now_s)
        {
            if (!map_command_payload_seen_) {
                map_command_payload_seen_ = true;
                last_map_command_payload_ = msg;
                map_command_last_packet_time_s_ = now_s;
                // 启动后首帧可能只是服务器对历史指令的1Hz重发，先建立基线而不触发。
                map_command_cluster_triggered_ = false;
                return false;
            }

            if (!sameMapCommandPayload(msg, last_map_command_payload_)) {
                last_map_command_payload_ = msg;
                map_command_last_packet_time_s_ = now_s;
                map_command_cluster_triggered_ = true;
                return true;
            }

            const double packet_gap_s = now_s - map_command_last_packet_time_s_;
            map_command_last_packet_time_s_ = now_s;

            if (packet_gap_s > kMapCommandBurstGapSeconds) {
                // 同内容的单个1Hz包先视为协议重发；若随后出现100ms连发，再确认是一次新触发。
                map_command_cluster_triggered_ = false;
                return false;
            }

            if (!map_command_cluster_triggered_) {
                map_command_cluster_triggered_ = true;
                return true;
            }
            return false;
        }

        double getBlackboardDouble(const std::string &key, double default_value) const
        {
            double value = default_value;
            if (!blackboard_utils::getValue(
                    blackboard_, key, value, "BlackboardManager")) {
                return default_value;
            }
            return value;
        }

        bool isCurrentPoseFresh() const
        {
            if (!waypoint_now_received_) {
                return false;
            }

            double stale_timeout = 0.50;
            if (!blackboard_->get("ul_pose_stale_timeout_s", stale_timeout)) {
                stale_timeout = 0.50;
            }
            return (node_->now() - last_waypoint_update_time_).seconds() <= stale_timeout;
        }

        void updateCurrentPoseValidity()
        {
            const bool pose_valid = isCurrentPoseFresh();
            blackboard_->set("waypoint_now_valid", pose_valid);

            if (pose_valid == last_waypoint_now_valid_) {
                return;
            }

            if (pose_valid) {
                RCLCPP_INFO(node_->get_logger(), "当前位姿恢复，重新启用中心相关点位判定");
            } else {
                RCLCPP_WARN(node_->get_logger(), "当前位姿已过期，暂停中心相关点位判定与 /vw 驻守");
            }
            last_waypoint_now_valid_ = pose_valid;
        }

        // 更新当前位置的方法
        void updateCurrentPosition()
        {
            try
            {
                // 查询 map 到 base_link 的变换
                geometry_msgs::msg::TransformStamped transform_stamped =
                    tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero, tf2::durationFromSec(0.05));

                // 创建 PoseStamped 消息
                geometry_msgs::msg::PoseStamped current_pose;
                current_pose.header.stamp = node_->now();
                current_pose.header.frame_id = "map";

                // 设置位置
                current_pose.pose.position.x = transform_stamped.transform.translation.x;
                current_pose.pose.position.y = transform_stamped.transform.translation.y;
                current_pose.pose.position.z = transform_stamped.transform.translation.z;

                // 设置方向
                current_pose.pose.orientation = transform_stamped.transform.rotation;

                // 将当前位置存储到黑板，命名为 waypoint_now
                blackboard_->set("waypoint_now", current_pose);
                last_waypoint_update_time_ = current_pose.header.stamp;
                waypoint_now_received_ = true;
                updateCurrentPoseValidity();

                // 打印调试信息（使用 DEBUG 级别避免太多日志）
                RCLCPP_DEBUG(node_->get_logger(),
                             "当前位置更新: (%.2f, %.2f, %.2f)",
                             current_pose.pose.position.x,
                             current_pose.pose.position.y,
                             current_pose.pose.position.z);
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN_THROTTLE(
                    node_->get_logger(),
                    *node_->get_clock(),
                    1000, // 每 1 秒最多警告一次
                    "无法获取当前位置: %s", ex.what());
                updateCurrentPoseValidity();
            }
        }
    };

BlackboardManager::BlackboardManager(
    rclcpp::Node::SharedPtr node,
    BT::Blackboard::Ptr blackboard)
    : impl_(std::make_unique<Impl>(std::move(node), std::move(blackboard)))
{
}

BlackboardManager::~BlackboardManager() = default;

BlackboardManager::BlackboardManager(BlackboardManager &&) noexcept = default;

BlackboardManager &BlackboardManager::operator=(BlackboardManager &&) noexcept = default;

void BlackboardManager::bb_manager_init()
{
    impl_->bb_manager_init();
}

void BlackboardManager::load_waypoints(const std::string &json_file_path)
{
    impl_->load_waypoints(json_file_path);
}

} // namespace sentry_nav_bt

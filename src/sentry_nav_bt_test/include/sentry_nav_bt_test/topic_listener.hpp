#ifndef SENTRY_NAV_BT_TEST_TOPIC_LISTENER_HPP_
#define SENTRY_NAV_BT_TEST_TOPIC_LISTENER_HPP_

#include <string>
#include <cmath>
#include <memory>
#include <vector>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/blackboard.h"

#include <fstream>
#include <nlohmann/json.hpp>
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
#include "rm_referee_msgs/msg/hurt_data.hpp"
#include "rm_referee_msgs/msg/event_data.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include "sentry_nav_bt_test/center_hold_vw_controller.hpp"

namespace sentry_nav_bt_test
{
    /**
     * @brief 黑板管理器类，简化自定义话题的订阅和黑板更新
     */
    class BlackboardManager
    {
    public:
        /**
         * @brief 构造函数
         * @param node ROS节点指针
         * @param blackboard 行为树黑板指针
         */
        BlackboardManager(rclcpp::Node::SharedPtr node, BT::Blackboard::Ptr blackboard)
            : node_(node), blackboard_(blackboard), mutex_()
        {
            RCLCPP_INFO(node_->get_logger(), "初始化黑板管理器");
        }

        /**
         * @brief 订阅自定义消息类型并更新到黑板
         * @param topic_name 话题名称
         * @param blackboard_key 黑板中的键名
         * @param queue_size 队列大小
         */
        template <typename MsgType>
        void subscribe(
            const std::string &topic_name,
            const std::string &blackboard_key,
            size_t queue_size = 10)
        {
            auto callback = [this, blackboard_key](const typename MsgType::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lock(mutex_);
                blackboard_->set(blackboard_key, *msg);
                RCLCPP_DEBUG(
                    node_->get_logger(),
                    "黑板更新: '%s' 从话题 '%s'",
                    blackboard_key.c_str(),
                    typeid(MsgType).name());
            };

            auto subscription = node_->create_subscription<MsgType>(
                topic_name, queue_size, callback);
            subscriptions_.push_back(subscription);

            RCLCPP_DEBUG(
                node_->get_logger(),
                "已添加话题监听: %s -> %s",
                topic_name.c_str(),
                blackboard_key.c_str());
        }

        /**
         * @brief 高级使用 - 带自定义转换处理的订阅
         * @param topic_name 话题名称
         * @param processor 自定义处理函数，接收消息并更新黑板
         * @param queue_size 队列大小
         */
        template <typename MsgType>
        void subscribeWithProcessor(
            const std::string &topic_name,
            std::function<void(const typename MsgType::SharedPtr, BT::Blackboard::Ptr)> processor,
            size_t queue_size = 10)
        {
            auto callback = [this, processor](const typename MsgType::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lock(mutex_);
                processor(msg, blackboard_);
            };

            auto subscription = node_->create_subscription<MsgType>(
                topic_name, queue_size, callback);
            subscriptions_.push_back(subscription);

            RCLCPP_DEBUG(
                node_->get_logger(),
                "已添加带处理器的话题监听: %s",
                topic_name.c_str());
        }

        // 在BlackboardManager类中添加新的订阅方法
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
            // 设置必要的黑板参数
            blackboard_->set<std::chrono::milliseconds>("bt_loop_duration", std::chrono::milliseconds(10));
            blackboard_->set<std::chrono::milliseconds>("wait_for_service_timeout", std::chrono::milliseconds(1000));
            blackboard_->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(1000));
            blackboard_->set<bool>("initial_pose_received", false);

            // 显式记录是否已经收到过裁判系统状态，避免用 game_progress 的默认值误判已连接
            blackboard_->set<bool>("game_status_received", false);
            blackboard_->set<int>("game_status_connected_logged", 0);

            // 初始化 UL/UC 状态，避免赛前条件检查因缺键刷 warning
            blackboard_->set<bool>("last_referee_tx_ok", false);
            blackboard_->set<int>("last_posture_request_target", -1);
            blackboard_->set<bool>("last_posture_request_sent", false);
            blackboard_->set<bool>("last_posture_request_tx_ok", false);
            blackboard_->set<bool>("last_posture_request_confirmed", false);
            blackboard_->set<bool>("last_posture_request_pending", false);
            blackboard_->set<std::string>("last_posture_request_result", "idle");
            blackboard_->set<double>("last_posture_request_time_s", -1.0);
            blackboard_->set<int>("posture_switch_cooldown_ms", 5000);
            blackboard_->set<int>("ul_initialized", 0);
            blackboard_->set<int>("uc_initialized", 0);
            blackboard_->set<int>("ul_retreat_active", 0);
            blackboard_->set<int>("ul_center_ready", 0);
            blackboard_->set<int>("uc_supply_active", 0);
            blackboard_->set<int>("uc_outpost_active", 0);
            blackboard_->set<int>("uc_normal_posture", 3);
            blackboard_->set<int>("center_gain_point_occupancy_status", 0);
            blackboard_->set<std::string>("ul_center_goal_name", "center_point");
            blackboard_->set<double>("ul_center_arrive_distance_threshold", 0.10);
            blackboard_->set<double>("ul_center_hold_distance_threshold", 0.50);
            blackboard_->set<double>("ul_center_hold_exit_distance_threshold", 0.55);
            blackboard_->set<int>("uc_fortress_hold_active", 0);
            blackboard_->set<std::string>("uc_fortress_goal_name", "fortress");
            blackboard_->set<double>("uc_fortress_hold_distance_threshold", 0.25);
            blackboard_->set<double>("uc_fortress_hold_exit_distance_threshold", 0.30);
            blackboard_->set<double>("ul_pose_stale_timeout_s", 0.50);
            blackboard_->set<bool>("waypoint_now_valid", false);
            blackboard_->set<uint32_t>("rfid_status", 0U);
            blackboard_->set<uint8_t>("rfid_status_2", 0U);
            blackboard_->set<int>("rfid_ally_fortress_detected", 0);
            blackboard_->set<int>("rfid_supply_zone_bit19_detected", 0);
            blackboard_->set<int>("rfid_supply_zone_bit20_detected", 0);
            blackboard_->set<int>("rfid_supply_zone_detected", 0);

            // 初始化 hurt_armor_id 为 -1
            blackboard_->set<int>("hurt_armor_id", -1);
            blackboard_->set<int>("hurt_armor", 0);
            blackboard_->set<int>("is_under_attack", 0);
            blackboard_->set<int>("in_rune_phase", 0);

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

            // 订阅己方机器人血量数据
            this->subscribeWithProcessorBestEffort<rm_referee_msgs::msg::GameRobotHP>(
                "/rm_referee/game_robot_hp",
                [this](const rm_referee_msgs::msg::GameRobotHP::SharedPtr msg, BT::Blackboard::Ptr bb)
                {
                    bb->set("ally_1_robot_hp", msg->ally_1_robot_hp);
                    bb->set("ally_2_robot_hp", msg->ally_2_robot_hp);
                    bb->set("ally_3_robot_hp", msg->ally_3_robot_hp);
                    bb->set("ally_4_robot_hp", msg->ally_4_robot_hp);
                    bb->set("ally_7_robot_hp", msg->ally_7_robot_hp);
                    bb->set("ally_outpost_hp", msg->ally_outpost_hp); // 前哨站
                    bb->set("ally_base_hp", msg->ally_base_hp);       // 基地
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

                    bb->set("large_rune_status", large_rune_status);
                    bb->set("small_rune_status", small_rune_status);
                    bb->set(
                        "center_gain_point_occupancy_status",
                        static_cast<int>(center_gain_point_occupancy_status));
                    RCLCPP_DEBUG(
                        node_->get_logger(),
                        "黑板更新: 大符状态=%d, 小符状态=%d, 中心增益点占领状态=%d",
                        large_rune_status,
                        small_rune_status,
                        center_gain_point_occupancy_status);
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

                    // 写入黑板
                    bb->set("exchanged_ammo", exchanged_ammo);
                    bb->set("can_confirm_resurrection", can_confirm_resurrection);
                    bb->set("can_buy_resurrection", can_buy_resurrection);
                    bb->set("buy_resurrection_cost", buy_resurrection_cost);

                    bb->set("current_posture", static_cast<int>(current_posture));
                    bb->set("can_activate_rune", can_activate_rune);

                    RCLCPP_DEBUG(node_->get_logger(),
                                 "哨兵信息: 姿态=%d, 可激活符=%d, 复活金币=%d",
                                 current_posture, can_activate_rune, buy_resurrection_cost);
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
                    bb->set("power_management_gimbal_output", msg->power_management_gimbal_output);
                    bb->set("power_management_chassis_output", msg->power_management_chassis_output);
                    bb->set("power_management_shooter_output", msg->power_management_shooter_output);
                    RCLCPP_DEBUG(
                        node_->get_logger(),
                        "黑板更新: '%s' 从话题 '%s'",
                        "robot_status",
                        "/rm_referee/robot_status");
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
            try
            {
                this->load_waypoints_from_json(json_file_path, blackboard_, node_);
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(node_->get_logger(), "加载目标点文件时出错: %s", e.what());
            }
        }

        // 从JSON文件加载路径点信息
        void load_waypoints_from_json(const std::string &json_file_path, BT::Blackboard::Ptr blackboard,
                                      std::shared_ptr<rclcpp::Node> node)
        {
            try
            {
                // 打开并读取JSON文件
                std::ifstream file(json_file_path);
                if (!file.is_open())
                {
                    RCLCPP_ERROR(node->get_logger(), "无法打开目标点文件: %s", json_file_path.c_str());
                    return;
                }

                // 解析JSON
                nlohmann::json waypoints_json;
                file >> waypoints_json;

                // 检查文件格式
                if (!waypoints_json.is_object() || !waypoints_json.contains("waypoints"))
                {
                    RCLCPP_ERROR(node->get_logger(), "JSON文件格式错误，应包含'waypoints'数组");
                    return;
                }

                // 获取并解析waypoints数组
                auto &waypoints_array = waypoints_json["waypoints"];
                if (!waypoints_array.is_array())
                {
                    RCLCPP_ERROR(node->get_logger(), "'waypoints'不是数组");
                    return;
                }

                int loaded_count = 0;

                for (const auto &wp : waypoints_array)
                {
                    if (!wp.is_object() || !wp.contains("name") || !wp.contains("x") || !wp.contains("y") || !wp.contains("yaw"))
                    {
                        RCLCPP_WARN(node->get_logger(), "跳过格式不正确的目标点");
                        continue;
                    }

                    // 创建PoseStamped消息
                    geometry_msgs::msg::PoseStamped pose;
                    pose.header.frame_id = "map";
                    pose.header.stamp = node->now();

                    // 设置位置
                    pose.pose.position.x = wp["x"];
                    pose.pose.position.y = wp["y"];
                    pose.pose.position.z = 0.0;

                    // 从偏航角计算四元数
                    double yaw = wp["yaw"];
                    tf2::Quaternion q;
                    q.setRPY(0, 0, yaw);
                    pose.pose.orientation.x = q.x();
                    pose.pose.orientation.y = q.y();
                    pose.pose.orientation.z = q.z();
                    pose.pose.orientation.w = q.w();

                    const std::string name = wp["name"];
                    blackboard->set("waypoint_" + name, pose);
                    loaded_count++;
                }

                RCLCPP_INFO(node->get_logger(), "成功加载 %d 个目标点", loaded_count);
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(node->get_logger(), "加载目标点文件时出错: %s", e.what());
            }
        }

    private:
        rclcpp::Node::SharedPtr node_;
        BT::Blackboard::Ptr blackboard_;
        std::vector<std::shared_ptr<rclcpp::SubscriptionBase>> subscriptions_;
        std::mutex mutex_;

        rclcpp::TimerBase::SharedPtr hurt_reset_timer_;
        std::mutex hurt_mutex_;

        std::shared_ptr<CenterHoldVwController> center_hold_vw_controller_;
        // TF
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        rclcpp::TimerBase::SharedPtr tf_timer_;
        rclcpp::Time last_waypoint_update_time_{0, 0, RCL_ROS_TIME};
        bool waypoint_now_received_ = false;
        bool last_waypoint_now_valid_ = false;

        double getBlackboardDouble(const std::string &key, double default_value) const
        {
            double value = default_value;
            if (!blackboard_->get(key, value)) {
                return default_value;
            }
            return value;
        }

        bool isCurrentPoseFresh() const
        {
            if (!waypoint_now_received_) {
                return false;
            }

            const double stale_timeout = getBlackboardDouble("ul_pose_stale_timeout_s", 0.50);
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
} // namespace sentry_nav_bt_test

#endif // SENTRY_NAV_BT_TEST_TOPIC_LISTENER_HPP_

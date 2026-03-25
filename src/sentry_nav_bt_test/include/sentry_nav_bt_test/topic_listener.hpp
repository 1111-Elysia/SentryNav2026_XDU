#ifndef SENTRY_NAV_BT_TEST_TOPIC_LISTENER_HPP_
#define SENTRY_NAV_BT_TEST_TOPIC_LISTENER_HPP_

#include <string>
#include <cmath>
#include <memory>
#include <vector>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/blackboard.h"

#include <fstream>
#include <nlohmann/json.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int16.hpp>
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
#include "sentry_msgs/msg/vw.hpp"

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

            // 初始化 UL 状态，避免赛前条件检查因缺键刷 warning
            blackboard_->set<bool>("last_referee_tx_ok", false);
            blackboard_->set<int>("ul_initialized", 0);
            blackboard_->set<int>("ul_retreat_active", 0);
            blackboard_->set<int>("ul_center_ready", 0);
            blackboard_->set<int>("center_gain_point_occupancy_status", 0);
            blackboard_->set<std::string>("ul_center_goal_name", "center_point");
            blackboard_->set<double>("ul_center_hold_distance_threshold", 0.50);
            blackboard_->set<double>("ul_pose_stale_timeout_s", 0.50);
            blackboard_->set<bool>("waypoint_now_valid", false);

            // 初始化 hurt_armor_id 为 -1
            blackboard_->set<int>("hurt_armor_id", -1);
            blackboard_->set<int>("is_under_attack", 0);

            // 初始化定时器
            hurt_reset_timer_ = node_->create_wall_timer(
                std::chrono::seconds(2),
                [this]()
                {
                    std::lock_guard<std::mutex> lock(hurt_mutex_);
                    
                    int attack_status = 0;
                    // 读取当前状态，如果是 1 则打印日志
                    if (blackboard_->get("is_under_attack", attack_status) && attack_status == 1)
                    {
                        RCLCPP_INFO(node_->get_logger(), "攻击停止 (2s超时)，受击状态复位");
                    }

                    blackboard_->set("is_under_attack", 0); 
                    blackboard_->set("hurt_armor_id", -1); 
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
                    // bb->set("enemy_1_robot_hp", msg->enemy_1_robot_hp);   可以改为敌方机器人，需要从雷达获取
                    // bb->set("enemy_2_robot_hp", msg->enemy_2_robot_hp);
                    // bb->set("enemy_3_robot_hp", msg->enemy_3_robot_hp);
                    // bb->set("enemy_4_robot_hp", msg->enemy_4_robot_hp);
                    // bb->set("enemy_7_robot_hp", msg->enemy_7_robot_hp);
                    // bb->set("enemy_outpost_hp", msg->enemy_outpost_hp);
                    // bb->set("enemy_base_hp", msg->enemy_base_hp);
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
                    //  Bit 19: 是否可以确认免费复活 (1=是)
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

                    bb->set("current_posture", current_posture);
                    bb->set("can_activate_rune", can_activate_rune);

                    publishDecodedSentryInfo(
                        exchanged_ammo,
                        can_confirm_resurrection,
                        can_buy_resurrection,
                        buy_resurrection_cost,
                        current_posture,
                        can_activate_rune);

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

            // 订阅空中支援数据
            // this->subscribeWithProcessorBestEffort<rm_referee_msgs::msg::AirSupportData>(
            //     "/rm_referee/air_support_data",
            //     [this](const rm_referee_msgs::msg::AirSupportData::SharedPtr msg, BT::Blackboard::Ptr bb) {
            //         bb->set("airforce_status", msg->airforce_status);
            //         bb->set("time_remain", msg->time_remain);
            //         RCLCPP_DEBUG(
            //             node_->get_logger(),
            //             "黑板更新: '%s' 从话题 '%s'",
            //             "air_support_data",
            //             "/rm_referee/air_support_data");
            //     }
            // );

            // 订阅弹丸允许信息  0x0208
            this->subscribeWithProcessorBestEffort<rm_referee_msgs::msg::ProjectileAllowance>(
                "/rm_referee/projectile_allowance",
                [this](const rm_referee_msgs::msg::ProjectileAllowance::SharedPtr msg, BT::Blackboard::Ptr bb)
                {
                    bb->set("projectile_allowance_17mm", msg->projectile_allowance_17mm);
                    bb->set("projectile_allowance_42mm", msg->projectile_allowance_42mm);
                    bb->set("remaining_gold_coin", msg->remaining_gold_coin);
                    // bb->set("projectile_allowance_fortress", msg->projectile_allowance_fortress); // 堡垒增益点提供的储备17mm弹丸允许发弹量
                    RCLCPP_DEBUG(
                        node_->get_logger(),
                        "黑板更新: '%s' 从话题 '%s'",
                        "projectile_allowance",
                        "/rm_referee/projectile_allowance");
                });

            // // 订阅RFID状态 （地形增益点位置，需要分字节解析，暂时不用）
            // this->subscribeWithProcessorBestEffort<rm_referee_msgs::msg::RFIDStatus>(
            //     "/rm_referee/rfid_status",
            //     [this](const rm_referee_msgs::msg::RFIDStatus::SharedPtr msg, BT::Blackboard::Ptr bb)
            //     {
            //         bb->set("rfid_status", msg->rfid_status);
            //         bb->set("rfid_status_2", msg->rfid_status_2);
            //         RCLCPP_DEBUG(
            //             node_->get_logger(),
            //             "黑板更新: '%s' 从话题 '%s'",
            //             "rfid_status",
            //             "/rm_referee/rfid_status");
            //     });

            // // 订阅自瞄目标信息（从自瞄系统获得）               ——还没沟通好
            // this->subscribeWithProcessorBestEffort<rmos_interfaces::msg::Target>(
            //     "/target",
            //     [this](const rmos_interfaces::msg::Target::SharedPtr msg, BT::Blackboard::Ptr bb)
            //     {
            //         bb->set("aim_id", msg->id);
            //         RCLCPP_DEBUG(
            //             node_->get_logger(),
            //             "黑板更新: '%s' 从话题 '%s'",
            //             "aim_id",
            //             "/target");
            //     });

            // 初始化调试发布者，便于直接 ros2 topic echo 查看解包结果
            exchanged_ammo_pub_ = node_->create_publisher<std_msgs::msg::UInt16>("/sentry_nav_bt_test/decoded_sentry_info/exchanged_ammo", 10);
            can_confirm_resurrection_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/sentry_nav_bt_test/decoded_sentry_info/can_confirm_resurrection", 10);
            can_buy_resurrection_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/sentry_nav_bt_test/decoded_sentry_info/can_buy_resurrection", 10);
            buy_resurrection_cost_pub_ = node_->create_publisher<std_msgs::msg::UInt16>("/sentry_nav_bt_test/decoded_sentry_info/buy_resurrection_cost", 10);
            current_posture_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/sentry_nav_bt_test/decoded_sentry_info/current_posture", 10);
            can_activate_rune_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/sentry_nav_bt_test/decoded_sentry_info/can_activate_rune", 10);

            // 初始化发布者 (发给底盘驱动)
            vw_pub_ = node_->create_publisher<sentry_msgs::msg::Vw>("/vw", 10);
            vw_timer_ = node_->create_wall_timer(
                std::chrono::milliseconds(100),
                [this]()
                { updateCenterHoldVwCommand(); });

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

                    // 重置定时器 (续命 2秒)
                    if (hurt_reset_timer_)
                        hurt_reset_timer_->reset();
                });

            // // 遥控器频道10订阅，用于模式切换  切换战术阶段的时间阈值   应该要删/改  旧协议
            // this->subscribeWithProcessorBestEffort<sbus_interface::msg::Sbus>(
            //     "/sbus",
            //     [this](const sbus_interface::msg::Sbus::SharedPtr msg, BT::Blackboard::Ptr bb)
            //     {
            //         bb->set("channel_10", msg->mapped_channels[10]);
            //         if (msg->mapped_channels[10] == 0)
            //         {
            //             bb->set("stage_one_time", 420);
            //         }
            //         else
            //         {
            //             bb->set("stage_one_time", 360);
            //         }
            //         RCLCPP_DEBUG(
            //             node_->get_logger(),
            //             "黑板更新: '%s' 从话题 '%s' : '%d'",
            //             "sbus",
            //             "/sbus",
            //             msg->mapped_channels[10]);
            //     });

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
            // if (node_->get_parameter("waypoints_file", waypoints_file) && !waypoints_file.empty()) {
            //     RCLCPP_INFO(node_->get_logger(), "从文件加载目标点: %s", waypoints_file.c_str());
            //     this->load_waypoints_from_json(waypoints_file, blackboard_, node_);
            // } else {
            //     RCLCPP_WARN(node_->get_logger(), "未指定目标点文件，将使用默认目标点或行为树定义的目标点");
            // }
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

                // 将目标点转换为PoseStamped并写入黑板
                std::vector<geometry_msgs::msg::PoseStamped> waypoints;
                int counter = 0;

                for (const auto &wp : waypoints_array)
                {
                    if (!wp.is_object() || !wp.contains("x") || !wp.contains("y") || !wp.contains("yaw"))
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

                    // 存储到向量和黑板中
                    waypoints.push_back(pose);

                    // 为每个目标点创建单独的黑板条目
                    std::string wp_name = "waypoint_" + std::to_string(counter);
                    blackboard->set(wp_name, pose);

                    // 如果有名称，也存储名称
                    if (wp.contains("name"))
                    {
                        std::string name = wp["name"];
                        blackboard->set("waypoint_name_" + std::to_string(counter), name);
                        // 同时添加按名称索引的目标点
                        blackboard->set("waypoint_" + name, pose);
                    }

                    counter++;
                }

                // 存储目标点总数
                blackboard->set("waypoints_count", counter);

                // 同时存储完整的目标点数组
                blackboard->set("waypoints", waypoints);

                RCLCPP_INFO(node->get_logger(), "成功加载 %d 个目标点", counter);
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

        rclcpp::Publisher<sentry_msgs::msg::Vw>::SharedPtr vw_pub_;
        rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr exchanged_ammo_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr can_confirm_resurrection_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr can_buy_resurrection_pub_;
        rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr buy_resurrection_cost_pub_;
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr current_posture_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr can_activate_rune_pub_;
        rclcpp::TimerBase::SharedPtr vw_timer_;
        bool last_center_hold_vw_active_ = false;
        bool vw_command_initialized_ = false;
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
            blackboard_->get(key, value);
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
                RCLCPP_INFO(node_->get_logger(), "[UL] 当前位姿恢复，重新启用中心相关点位判定");
            } else {
                RCLCPP_WARN(node_->get_logger(), "[UL] 当前位姿已过期，暂停中心相关点位判定与 /vw 驻守");
            }
            last_waypoint_now_valid_ = pose_valid;
        }

        bool isNearWaypoint(const std::string &waypoint_name, double threshold = 0.50) const
        {
            if (!isCurrentPoseFresh()) {
                return false;
            }
            geometry_msgs::msg::PoseStamped current_pose;
            geometry_msgs::msg::PoseStamped target_pose;

            if (!blackboard_->get("waypoint_now", current_pose)) {
                return false;
            }

            const std::string waypoint_key = "waypoint_" + waypoint_name;
            if (!blackboard_->get(waypoint_key, target_pose)) {
                return false;
            }

            const double dx = target_pose.pose.position.x - current_pose.pose.position.x;
            const double dy = target_pose.pose.position.y - current_pose.pose.position.y;
            return std::hypot(dx, dy) <= threshold;
        }

        bool isNearCurrentCenterGoal(double threshold = -1.0) const
        {
            if (threshold < 0.0) {
                threshold = getBlackboardDouble("ul_center_hold_distance_threshold", 0.50);
            }

            std::string goal_name = "center_point";
            blackboard_->get("ul_center_goal_name", goal_name);
            return isNearWaypoint(goal_name, threshold);
        }

        void publishDecodedSentryInfo(
            uint16_t exchanged_ammo,
            bool can_confirm_resurrection,
            bool can_buy_resurrection,
            uint16_t buy_resurrection_cost,
            uint8_t current_posture,
            bool can_activate_rune)
        {
            if (exchanged_ammo_pub_) {
                std_msgs::msg::UInt16 msg;
                msg.data = exchanged_ammo;
                exchanged_ammo_pub_->publish(msg);
            }

            if (can_confirm_resurrection_pub_) {
                std_msgs::msg::Bool msg;
                msg.data = can_confirm_resurrection;
                can_confirm_resurrection_pub_->publish(msg);
            }

            if (can_buy_resurrection_pub_) {
                std_msgs::msg::Bool msg;
                msg.data = can_buy_resurrection;
                can_buy_resurrection_pub_->publish(msg);
            }

            if (buy_resurrection_cost_pub_) {
                std_msgs::msg::UInt16 msg;
                msg.data = buy_resurrection_cost;
                buy_resurrection_cost_pub_->publish(msg);
            }

            if (current_posture_pub_) {
                std_msgs::msg::UInt8 msg;
                msg.data = current_posture;
                current_posture_pub_->publish(msg);
            }

            if (can_activate_rune_pub_) {
                std_msgs::msg::Bool msg;
                msg.data = can_activate_rune;
                can_activate_rune_pub_->publish(msg);
            }
        }

        bool isCenterHoldActive() const
        {
            int center_ready = 0;
            int retreat_active = 0;
            int ul_initialized = 0;
            uint16_t current_hp = 0;
            uint8_t game_progress = 0;

            const bool center_ready_ok =
                blackboard_->get("ul_center_ready", center_ready) && center_ready == 1;
            const bool retreat_inactive =
                blackboard_->get("ul_retreat_active", retreat_active) && retreat_active == 0;
            const bool initialized =
                blackboard_->get("ul_initialized", ul_initialized) && ul_initialized == 1;
            const bool hp_ok =
                blackboard_->get("current_hp", current_hp) && current_hp >= 150U;
            const bool match_started =
                blackboard_->get("game_progress", game_progress) && game_progress > 3U;
            const bool current_center_goal_nearby = isNearCurrentCenterGoal();

            return match_started && initialized && retreat_inactive && hp_ok &&
                   center_ready_ok && current_center_goal_nearby;
        }

        void publishVwCommand(float value)
        {
            if (!vw_pub_) {
                return;
            }

            sentry_msgs::msg::Vw msg;
            msg.vw = value;
            vw_pub_->publish(msg);
        }

        void updateCenterHoldVwCommand()
        {
            const bool center_hold_active = isCenterHoldActive();

            if (!vw_command_initialized_)
            {
                publishVwCommand(center_hold_active ? 1.0f : 0.0f);
                vw_command_initialized_ = true;
                last_center_hold_vw_active_ = center_hold_active;
                return;
            }

            if (center_hold_active)
            {
                if (!last_center_hold_vw_active_)
                {
                    RCLCPP_INFO(node_->get_logger(), "[UL] 中心驻守激活，开始持续发布 /vw = 1");
                }
                publishVwCommand(1.0f);
            }
            else if (last_center_hold_vw_active_)
            {
                RCLCPP_INFO(node_->get_logger(), "[UL] 退出中心驻守，停止持续发布 /vw");
                publishVwCommand(0.0f);
            }

            last_center_hold_vw_active_ = center_hold_active;
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

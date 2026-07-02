#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sentry_msgs/msg/vw.hpp>
#include <sentry_msgs/msg/armor_presence.hpp>
#include <sentry_msgs/msg/scan_mode.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include "rm_referee_msgs/msg/game_status.hpp"
#include <rclcpp/qos.hpp>

#include <librm.hpp>

using rm::hal::Can;

#include <cstdint>
#include <mutex>
#include <chrono>
#include <memory>
#include <algorithm>

using namespace std::chrono_literals;

class CanCommNode : public rclcpp::Node
{
public:
    CanCommNode()
    : Node("can_send")
    {
        // 参数声明
        this->declare_parameter<std::string>("port", "can2");
        this->declare_parameter<int>("send_frequency", 500);
        // 发送用的 CAN ID 参数（单帧发送）
        this->declare_parameter<int>("id_scan", 0x190);
        this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
        this->declare_parameter<std::string>("vw_topic", "/vw");
        this->declare_parameter<std::string>("scan_mod_type_topic", "/scan_mod_type");
        this->declare_parameter<std::string>("NLJG_mode_type_topic", "/NLJG_mode_type");
        this->declare_parameter<std::string>("outpost_mode_type_topic", "/outpost_mode_type");
        this->declare_parameter<std::string>("all_detect_topic", "/detector/armor_presence");
        this->declare_parameter<std::string>("game_status_topic", "/rm_referee/game_status");
        this->declare_parameter<std::string>("target_yaw_topic", "/target_yaw");
        this->declare_parameter<double>("cmd_vel_timeout_s", 0.1);
        // target_yaw 无超时：电控靠值不再更新来判定停止
        this->declare_parameter<double>("vw_timeout_s", 0.1);

        // 读取参数
        std::string port      = this->get_parameter("port").as_string();
        int send_freq  = this->get_parameter("send_frequency").as_int();
        int id_scan_int = this->get_parameter("id_scan").as_int();
        id_scan_ = static_cast<uint32_t>(id_scan_int);

        std::string cmd_vel_topic       = this->get_parameter("cmd_vel_topic").as_string();
        std::string vw_topic            = this->get_parameter("vw_topic").as_string();
        std::string scan_mod_type_topic = this->get_parameter("scan_mod_type_topic").as_string();
        std::string NLJG_mode_type_topic = this->get_parameter("NLJG_mode_type_topic").as_string();
        std::string outpost_mode_type_topic = this->get_parameter("outpost_mode_type_topic").as_string();
        std::string all_detect_topic    = this->get_parameter("all_detect_topic").as_string();
        std::string game_status_topic   = this->get_parameter("game_status_topic").as_string();
        std::string target_yaw_topic   = this->get_parameter("target_yaw_topic").as_string();
        cmd_vel_timeout_s_ = this->get_parameter("cmd_vel_timeout_s").as_double();
        vw_timeout_s_ = this->get_parameter("vw_timeout_s").as_double();

        // 打开 CAN 设备
        try {
            can_ = std::make_unique<Can>(port.c_str());
            can_->Begin();
            RCLCPP_INFO(this->get_logger(), "✓ CAN 打开: %s", port.c_str());
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "✗ CAN 打开失败: %s", port.c_str());
            rclcpp::shutdown();
            return;
        }

        // 订阅话题
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic, 10,
            std::bind(&CanCommNode::cmdVelCallback, this, std::placeholders::_1));

        vw_sub_ = this->create_subscription<sentry_msgs::msg::Vw>(
            vw_topic, 10,
            [this](const sentry_msgs::msg::Vw::SharedPtr m) {
                std::lock_guard<std::mutex> lk(mutex_);
                if (!vw_received_once_) {
                    vw_received_once_ = true;
                }
                vw_ = m->vw;
                last_vw_msg_time_ = this->now();
            });

        scan_mod_sub_ = this->create_subscription<sentry_msgs::msg::ScanMode>(
            scan_mod_type_topic, 10,
            [this](const sentry_msgs::msg::ScanMode::SharedPtr m) {
                std::lock_guard<std::mutex> lk(mutex_);
                scan_mod_type_ = m->scan_mod_type;
                scan_mod_type_topic_received_ = true;
            });

        NLJG_mode_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            NLJG_mode_type_topic, 10,
            [this](const std_msgs::msg::Bool::SharedPtr m) {
                std::lock_guard<std::mutex> lk(mutex_);
                NLJG_mode_ = m->data;
            });

        outpost_mode_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            outpost_mode_type_topic, 10,
            [this](const std_msgs::msg::Bool::SharedPtr m) {
                std::lock_guard<std::mutex> lk(mutex_);
                outpost_mode_ = m->data;
            });

        game_status_sub_ = this->create_subscription<rm_referee_msgs::msg::GameStatus>(
            game_status_topic,
            rclcpp::SensorDataQoS(),   
            std::bind(&CanCommNode::gamestatusCallback, this, std::placeholders::_1));

        armor_presence_sub_ = this->create_subscription<sentry_msgs::msg::ArmorPresence>(
            all_detect_topic, 10,
            [this](const sentry_msgs::msg::ArmorPresence::SharedPtr m) {
                std::lock_guard<std::mutex> lk(mutex_);
                armor_left_   = m->left;
                armor_behind_ = m->behind;
                armor_right_  = m->right;
            });

        target_yaw_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            target_yaw_topic, 10,
            [this](const std_msgs::msg::Float32::SharedPtr m) {
                std::lock_guard<std::mutex> lk(mutex_);
                target_yaw_ = m->data;
            });

        // 定时发送 CAN 帧
        if (send_freq <= 0) send_freq = 1;
        int period_ms = 1000 / send_freq;
        last_log_ = this->now();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&CanCommNode::sendFrame, this));

        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "CAN 通信节点初始化完成");
        RCLCPP_INFO(this->get_logger(), "========================================");
    }

    ~CanCommNode() override = default;

private:
    // cmd_vel 回调，仅更新 vx_ / vy_
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        vx_   = static_cast<float>(msg->linear.x);
        vy_   = static_cast<float>(msg->linear.y);
        cmd_vel_received_once_ = true;
        last_cmd_vel_msg_time_ = this->now();
    }

    void gamestatusCallback(const rm_referee_msgs::msg::GameStatus::SharedPtr msg)
    {
        if (!msg) return;
        std::lock_guard<std::mutex> lock(mutex_);
        game_progress_ = msg->game_progress;
        if (!scan_mod_type_topic_received_) {
            scan_mod_type_ = (msg->game_progress == 4);
        }
    }

    void sendFrame()
    {
        if (!can_) return;

        float vx, vy, target_yaw, vw;
        bool scan;
        bool NLJG_mode;
        bool outpost_mode;
        uint8_t left = 0;
        uint8_t behind = 0;
        uint8_t right = 0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            vx         = vx_;
            vy         = vy_;
            target_yaw = target_yaw_;
            vw         = vw_;
            scan       = scan_mod_type_;
            NLJG_mode  = NLJG_mode_;
            outpost_mode = outpost_mode_;
            left   = armor_left_;
            behind = armor_behind_;
            right  = armor_right_;

            const auto now = this->now();
            const bool cmd_vel_fresh =
                cmd_vel_received_once_ &&
                (now - last_cmd_vel_msg_time_).seconds() <= cmd_vel_timeout_s_;
            if (!cmd_vel_fresh) {
                vx = 0.0f;
                vy = 0.0f;
            }

            const bool vw_fresh =
                vw_received_once_ &&
                (now - last_vw_msg_time_).seconds() <= vw_timeout_s_;
            if (!vw_fresh) {
                vw = 0.0f;
            }
        }

        // 限幅 lambda
        auto clamp = [](float v, float min_v, float max_v) {
            return std::max(std::min(v, max_v), min_v);
        };

        // vx/vy/vw: 归一化到 [-1, 1] 再乘 127 → int8_t
        int8_t vx_q = static_cast<int8_t>(clamp(vx, -1.0f, 1.0f) * 127.0f);
        int8_t vy_q = static_cast<int8_t>(clamp(vy, -1.0f, 1.0f) * 127.0f);
        int8_t vw_q = static_cast<int8_t>(clamp(vw, -1.0f, 1.0f) * 127.0f);

        // target_yaw: 限幅 [-180, 180] 再乘 100 → int16_t, big-endian
        int16_t target_yaw_q = static_cast<int16_t>(clamp(target_yaw, -180.0f, 180.0f) * 100.0f);

        // 单帧 6 字节，ID 0x190
        uint8_t data[6] = {0};

        // byte [0]: vx (int8_t)
        data[0] = static_cast<uint8_t>(vx_q);

        // byte [1]: vy (int8_t)
        data[1] = static_cast<uint8_t>(vy_q);

        // byte [2-3]: target_yaw (int16_t, big-endian, ×100, 范围[-180,180])
        data[2] = (target_yaw_q >> 8) & 0xFF;
        data[3] = target_yaw_q & 0xFF;

        // byte [4]: 低4位 = left|behind|right, 高4位 = scan_mode|NLJG_mode|outpost_mode
        data[4] = (left ? 1 : 0)
                | ((behind ? 1 : 0) << 1)
                | ((right ? 1 : 0) << 2)
                | ((scan ? 1 : 0) << 4)
                | ((NLJG_mode ? 1 : 0) << 5)
                | ((outpost_mode ? 1 : 0) << 6);

        // byte [5]: vw (int8_t)
        data[5] = static_cast<uint8_t>(vw_q);

        try {
            can_->Write(id_scan_, data, sizeof(data));
        } catch (const std::exception &e) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "CAN Write failed: %s", e.what());
        }

        // 频率日志
        send_count_++;
        auto now = this->now();
        double dt = (now - last_log_).seconds();
        if (dt >= 1.0) {
            double freq = send_count_ / dt;
            RCLCPP_INFO(this->get_logger(),
                        "CAN发送频率: %.1f Hz | vx=%.3f vy=%.3f target_yaw=%.3f vw=%.3f scan=%u NLJG_mode=%u outpost_mode=%u left=%u behind=%u right=%u",
                        freq, vx, vy, target_yaw, vw,
                        static_cast<unsigned>(scan),
                        static_cast<unsigned>(NLJG_mode),
                        static_cast<unsigned>(outpost_mode),
                        static_cast<unsigned>(left),
                        static_cast<unsigned>(behind),
                        static_cast<unsigned>(right));
            send_count_ = 0;
            last_log_ = now;
        }
    }

private:
    std::unique_ptr<Can> can_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr       cmd_vel_sub_;
    rclcpp::Subscription<sentry_msgs::msg::Vw>::SharedPtr            vw_sub_;
    rclcpp::Subscription<sentry_msgs::msg::ScanMode>::SharedPtr      scan_mod_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr             NLJG_mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr             outpost_mode_sub_;
    rclcpp::Subscription<rm_referee_msgs::msg::GameStatus>::SharedPtr game_status_sub_;
    rclcpp::Subscription<sentry_msgs::msg::ArmorPresence>::SharedPtr armor_presence_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr            target_yaw_sub_;
    rclcpp::TimerBase::SharedPtr                                     timer_;

    std::mutex mutex_;
    float vx_ = 0.0f, vy_ = 0.0f, target_yaw_ = 0.0f;
    float vw_ = 0.0f;
    bool vw_received_once_ = false;
    uint8_t game_progress_ = 0;
    bool scan_mod_type_ = false;
    bool scan_mod_type_topic_received_ = false;
    bool NLJG_mode_ = false;
    bool outpost_mode_ = false;
    uint8_t armor_left_ = 0, armor_behind_ = 0, armor_right_ = 0;

    bool cmd_vel_received_once_ = false;
    rclcpp::Time last_cmd_vel_msg_time_;
    rclcpp::Time last_vw_msg_time_;
    double cmd_vel_timeout_s_ = 0.5;
    double vw_timeout_s_ = 0.5;

    uint32_t id_scan_ = 0x190;

    size_t send_count_ = 0;
    rclcpp::Time last_log_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CanCommNode>());
    rclcpp::shutdown();
    return 0;
}
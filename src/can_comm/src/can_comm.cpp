#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sentry_msgs/msg/vw.hpp>
#include <sentry_msgs/msg/armor_presence.hpp>
#include "rm_referee_msgs/msg/hurt_data.hpp"
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
    : Node("can_comm_node")
    {
        // 参数声明
        this->declare_parameter<std::string>("port", "can2");
        this->declare_parameter<int>("send_frequency", 500);
        // 发送用的两个 ID 参数
        this->declare_parameter<int>("id_xyz", 0x180);
        this->declare_parameter<int>("id_scan", 0x190);
        this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
        this->declare_parameter<std::string>("vw_topic", "/vw");
        this->declare_parameter<std::string>("all_detect_topic", "/detector/armor_presence");
        this->declare_parameter<std::string>("hurt_data_topic", "/rm_referee/hurt_data");
        this->declare_parameter<std::string>("game_status_topic", "/rm_referee/game_status");
        this->declare_parameter<double>("cmd_vel_timeout_s", 0.1);
        this->declare_parameter<double>("vw_timeout_s", 0.1);

        // 读取参数
        std::string port      = this->get_parameter("port").as_string();
        int send_freq  = this->get_parameter("send_frequency").as_int();
        int id_xyz_int = this->get_parameter("id_xyz").as_int();
        int id_scan_int = this->get_parameter("id_scan").as_int();
        id_xyz_  = static_cast<uint32_t>(id_xyz_int);
        id_scan_ = static_cast<uint32_t>(id_scan_int);

        std::string cmd_vel_topic       = this->get_parameter("cmd_vel_topic").as_string();
        std::string vw_topic            = this->get_parameter("vw_topic").as_string();
        std::string all_detect_topic    = this->get_parameter("all_detect_topic").as_string();
        std::string hurt_data_topic     = this->get_parameter("hurt_data_topic").as_string();
        std::string game_status_topic   = this->get_parameter("game_status_topic").as_string();
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
                // 新的 /vw 指令到达后，退出默认值覆盖模式
                vw_default_override_active_ = false;
            });

        // game_status_sub_ = this->create_subscription<rm_referee_msgs::msg::GameStatus>(
        //     game_status_topic, 10,
        //     [this](const rm_referee_msgs::msg::GameStatus::SharedPtr m) {
        //         std::lock_guard<std::mutex> lk(mutex_);
        //         game_progress_ = m->game_progress;
        //         scan_mod_type_ = (m->game_progress == 4);
        //     });

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

        hurt_sub_ = this->create_subscription<rm_referee_msgs::msg::HurtData>(
            hurt_data_topic,
            rclcpp::SensorDataQoS(),   // BestEffort + small queue
            std::bind(&CanCommNode::hurtCallback, this, std::placeholders::_1));
    
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
    // 新增：cmd_vel 回调，更新 vx_ / vy_ / vyaw_
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        vx_   = static_cast<float>(msg->linear.x);
        vy_   = static_cast<float>(msg->linear.y);
        vyaw_ = static_cast<float>(msg->angular.z);
        cmd_vel_received_once_ = true;
        last_cmd_vel_msg_time_ = this->now();
    }

    void hurtCallback(const rm_referee_msgs::msg::HurtData::SharedPtr msg)
    {
        if (!msg) return;
        if (msg->hp_deduction_reason == 0) {
            std::lock_guard<std::mutex> lock(mutex_);
            hurt_active_ = true;
            hurt_start_time_ = this->now();

            vw_ = 1.0f;
        }
    }

    void gamestatusCallback(const rm_referee_msgs::msg::GameStatus::SharedPtr msg)
    {
        if (!msg) return;
        game_progress_ = msg->game_progress;
        scan_mod_type_ = (msg->game_progress == 4);
    }

    void sendFrame()
    {
        if (!can_) return;

        float vx, vy, vyaw, vw;
        bool scan;
        uint8_t left = 0;
        uint8_t behind = 0;
        uint8_t right = 0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            vx    = vx_;
            vy    = vy_;
            vyaw  = vyaw_;
            vw    = vw_;
            scan  = scan_mod_type_;
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
                vyaw = 0.0f;
            }

            if (hurt_active_) {
                double dt = (now - hurt_start_time_).seconds();
                if (dt < 5.0) {
                    vw = 1.0f;
                    vw_ = vw;
                } else {
                    hurt_active_ = false;
                    vw = vw_received_once_ ? vw_default_after_first_msg_ : vw_default_before_first_msg_;
                    vw_ = vw;
                    // 受击结束后切到 0.3 的默认值时，其优先级高于 vw 超时置零
                    vw_default_override_active_ = vw_received_once_;
                }
            } else {
                if (vw_default_override_active_) {
                    vw = vw_default_after_first_msg_;
                    vw_ = vw;
                } else {
                    const bool vw_fresh =
                        vw_received_once_ &&
                        (now - last_vw_msg_time_).seconds() <= vw_timeout_s_;
                    if (!vw_fresh) {
                        vw = 0.0f;
                    }
                }
            }

            // 运动和装甲检测位门控直接依据 game_status 话题内容
            if (game_progress_ != 4) {
                vx = 0.0f;
                vy = 0.0f;
                vyaw = 0.0f;
                left = 0;
                behind = 0;
                right = 0;
            }
        }

        // 限幅 lambda
        auto clamp = [](float v, float min_v, float max_v) {
            return std::max(std::min(v, max_v), min_v);
        };

        // 限幅 [-32.767, 32.767] 并乘 1000 转 int16
        int16_t vx_q   = static_cast<int16_t>(clamp(vx,   -32.767f, 32.767f) * 1000.0f);
        int16_t vy_q   = static_cast<int16_t>(clamp(vy,   -32.767f, 32.767f) * 1000.0f);
        int16_t vyaw_q = static_cast<int16_t>(clamp(vyaw,-32.767f, 32.767f) * 1000.0f);
        int16_t vw_q   = static_cast<int16_t>(clamp(vw,   -32.767f, 32.767f) * 1000.0f);

        uint8_t data_xyz[8];
        data_xyz[0] = (vx_q >> 8) & 0xFF;
        data_xyz[1] = vx_q & 0xFF;

        data_xyz[2] = (vy_q >> 8) & 0xFF;
        data_xyz[3] = vy_q & 0xFF;

        data_xyz[4] = (vw_q >> 8) & 0xFF;
        data_xyz[5] = vw_q & 0xFF;

        data_xyz[6] = (vyaw_q >> 8) & 0xFF;
        data_xyz[7] = vyaw_q & 0xFF;

        can_->Write(id_xyz_, data_xyz, sizeof(data_xyz));

        // 发送 scan mode + ArmorPresence
        uint8_t data_scan[8] = {0};
        data_scan[0] = scan ? 1 : 0;
        data_scan[1] = left | (behind << 1) | (right << 2);
        can_->Write(id_scan_, data_scan, sizeof(data_scan));

        // 频率日志
        send_count_++;
        auto now = this->now();
        double dt = (now - last_log_).seconds();
        if (dt >= 1.0) {
            double freq = send_count_ / dt;
            RCLCPP_INFO(this->get_logger(),
                        "CAN发送频率: %.1f Hz | vx=%.3f vy=%.3f vyaw=%.3f vw=%.3f scan=%u left=%u behind=%u right=%u",
                        freq, vx, vy, vyaw, vw,
                        static_cast<unsigned>(scan),
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
    rclcpp::Subscription<rm_referee_msgs::msg::GameStatus>::SharedPtr game_status_sub_;
    rclcpp::Subscription<sentry_msgs::msg::ArmorPresence>::SharedPtr armor_presence_sub_;
    rclcpp::Subscription<rm_referee_msgs::msg::HurtData>::SharedPtr hurt_sub_;
    rclcpp::TimerBase::SharedPtr                                     timer_;

    std::mutex mutex_;
    float vx_ = 0.0f, vy_ = 0.0f, vyaw_ = 0.0f;float vw_ = 0.0f;
    float vw_default_before_first_msg_ = 0.0f;
    float vw_default_after_first_msg_ = 0.3f;
    bool vw_received_once_ = false;
    uint8_t game_progress_ = 0;
    bool scan_mod_type_ = false;
    uint8_t armor_left_ = 0, armor_behind_ = 0, armor_right_ = 0;
    bool hurt_active_ = false;
    rclcpp::Time hurt_start_time_;

    bool cmd_vel_received_once_ = false;
    rclcpp::Time last_cmd_vel_msg_time_;
    rclcpp::Time last_vw_msg_time_;
    double cmd_vel_timeout_s_ = 0.5;
    double vw_timeout_s_ = 0.5;
    bool vw_default_override_active_ = false;

    uint32_t id_xyz_  = 0x180;
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
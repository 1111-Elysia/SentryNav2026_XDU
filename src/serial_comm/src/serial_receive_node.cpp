#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp>
#include <cstdint>
#include <vector>
#include <sentry_msgs/msg/hurt_armor.hpp>
#include <sentry_msgs/msg/hp.hpp>
#include <sentry_msgs/msg/match_stage.hpp>
#include <sentry_msgs/msg/remain_time.hpp>
#include <sentry_msgs/msg/remain17mm.hpp>
#include <sentry_msgs/msg/color.hpp>

#pragma pack(push, 1)
typedef struct {
    uint8_t  SOF;          // 0x55 (与发送端一致)
    uint8_t  ID;           // 0x04 (与发送端一致)
    uint8_t  hurt_armor;   // 受击装甲板
    uint16_t hp;           // 血量
    uint8_t  match_stage;  // 比赛阶段
    uint16_t remain_time;  // 剩余时间
    uint16_t remain_17mm;  // 剩余弹药
    bool     is_blue;      // 红蓝方
    uint8_t  _EOF;         // 0xFF (与发送端一致)
} EctrlToNucFrame;
#pragma pack(pop)

class SerialReceiveNode : public rclcpp::Node
{
public:
    SerialReceiveNode() 
        : Node("serial_receive_node"), 
          io_context_(), 
          serial_port_(io_context_)
    {
        this->declare_parameter<std::string>("port", "/dev/ttyACM0");
        this->declare_parameter<int>("baudrate", 115200);

        // 声明并读取发布话题参数
        this->declare_parameter<std::string>("hurt_armor_topic", "/hurt_armor");
        this->declare_parameter<std::string>("hp_topic", "/hp");
        this->declare_parameter<std::string>("match_stage_topic", "/match_stage");
        this->declare_parameter<std::string>("remain_time_topic", "/remain_time");
        this->declare_parameter<std::string>("remain_17mm_topic", "/remain_17mm");
        this->declare_parameter<std::string>("robot_color_topic", "/robot_color");

        std::string port = this->get_parameter("port").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();

        std::string hurt_topic = this->get_parameter("hurt_armor_topic").as_string();
        std::string hp_topic = this->get_parameter("hp_topic").as_string();
        std::string match_topic = this->get_parameter("match_stage_topic").as_string();
        std::string remain_time_topic = this->get_parameter("remain_time_topic").as_string();
        std::string remain_17mm_topic = this->get_parameter("remain_17mm_topic").as_string();
        std::string color_topic = this->get_parameter("robot_color_topic").as_string();

        try {
            serial_port_.open(port);
            serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
            RCLCPP_INFO(this->get_logger(), "✓ 接收串口打开: %s @ %d", port.c_str(), baudrate);
        } catch (const boost::system::system_error& e) {
            RCLCPP_ERROR(this->get_logger(), "✗ 串口失败: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        hurt_armor_pub_ = this->create_publisher<sentry_msgs::msg::HurtArmor>(hurt_topic, 10);
        hp_pub_         = this->create_publisher<sentry_msgs::msg::Hp>(hp_topic, 10);
        match_stage_pub_= this->create_publisher<sentry_msgs::msg::MatchStage>(match_topic, 10);
        remain_time_pub_= this->create_publisher<sentry_msgs::msg::RemainTime>(remain_time_topic, 10);
        remain_17mm_pub_= this->create_publisher<sentry_msgs::msg::Remain17mm>(remain_17mm_topic, 10);
        color_pub_      = this->create_publisher<sentry_msgs::msg::Color>(color_topic, 10);

        startReceive();
        
        io_thread_ = std::thread([this]() { io_context_.run(); });
        
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "串口接收节点初始化完成");
        RCLCPP_INFO(this->get_logger(), "========================================");
    }

    ~SerialReceiveNode()
    {
        io_context_.stop();
        if (io_thread_.joinable()) io_thread_.join();
        if (serial_port_.is_open()) serial_port_.close();
    }

private:
    void startReceive()
    {
        boost::asio::async_read(
            serial_port_,
            boost::asio::buffer(&rx_buffer_, sizeof(EctrlToNucFrame)),
            [this](const boost::system::error_code& ec, std::size_t bytes) {
                if (!ec && bytes == sizeof(EctrlToNucFrame)) {
                    parseFrame();
                } else if (ec) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "读取错误: %s", ec.message().c_str());
                }
                startReceive();
            });
    }

    void parseFrame()
    {
        if (rx_buffer_.SOF != 0x55 || rx_buffer_.ID != 0x04 || rx_buffer_._EOF != 0xFF) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "帧校验失败 SOF=0x%02X ID=0x%02X EOF=0x%02X", 
                rx_buffer_.SOF, rx_buffer_.ID, rx_buffer_._EOF);
            return;
        }

        sentry_msgs::msg::HurtArmor hurt_msg;
        hurt_msg.hurt_armor = rx_buffer_.hurt_armor;
        hurt_armor_pub_->publish(hurt_msg);

        sentry_msgs::msg::Hp hp_msg;
        hp_msg.hp = rx_buffer_.hp;
        hp_pub_->publish(hp_msg);

        sentry_msgs::msg::MatchStage stage_msg;
        stage_msg.match_stage = rx_buffer_.match_stage;
        match_stage_pub_->publish(stage_msg);

        sentry_msgs::msg::RemainTime time_msg;
        time_msg.remain_time = rx_buffer_.remain_time;
        remain_time_pub_->publish(time_msg);

        sentry_msgs::msg::Remain17mm ammo_msg;
        ammo_msg.remain_17mm = rx_buffer_.remain_17mm;
        remain_17mm_pub_->publish(ammo_msg);

        sentry_msgs::msg::Color color_msg;
        color_msg.is_blue = rx_buffer_.is_blue;
        color_pub_->publish(color_msg);

        recv_count_++;
        auto now = this->now();
        if ((now - last_log_).seconds() >= 1.0) {
            double freq = recv_count_ / (now - last_log_).seconds();
            RCLCPP_INFO(this->get_logger(),
                "接收: %.1f Hz | hurt=%u hp=%u stage=%u time=%u ammo=%u blue=%u",
                freq, rx_buffer_.hurt_armor, rx_buffer_.hp, rx_buffer_.match_stage,
                rx_buffer_.remain_time, rx_buffer_.remain_17mm, (unsigned)rx_buffer_.is_blue);
            recv_count_ = 0;
            last_log_ = now;
        }
    }

private:
    boost::asio::io_context io_context_;
    boost::asio::serial_port serial_port_;
    std::thread io_thread_;

    rclcpp::Publisher<sentry_msgs::msg::HurtArmor>::SharedPtr hurt_armor_pub_;
    rclcpp::Publisher<sentry_msgs::msg::Hp>::SharedPtr hp_pub_;
    rclcpp::Publisher<sentry_msgs::msg::MatchStage>::SharedPtr match_stage_pub_;
    rclcpp::Publisher<sentry_msgs::msg::RemainTime>::SharedPtr remain_time_pub_;
    rclcpp::Publisher<sentry_msgs::msg::Remain17mm>::SharedPtr remain_17mm_pub_;
    rclcpp::Publisher<sentry_msgs::msg::Color>::SharedPtr color_pub_;

    EctrlToNucFrame rx_buffer_;
    size_t recv_count_ = 0;
    rclcpp::Time last_log_{this->now()};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialReceiveNode>());
    rclcpp::shutdown();
    return 0;
}

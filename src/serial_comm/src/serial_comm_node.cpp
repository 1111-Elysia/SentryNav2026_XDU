#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <boost/asio.hpp>
#include <cstdint>
#include <mutex>
#include <chrono>

#pragma pack(push, 1)
typedef struct {
    uint8_t SOF;
    uint8_t ID;
    float vx;
    float vy;
    float vyaw;
    uint8_t _EOF;
} NucControlFrame;
#pragma pack(pop)

class SerialCommNode : public rclcpp::Node
{
public:
    SerialCommNode() : Node("serial_comm_node"), io_context_(), serial_port_(io_context_)
    {
        this->declare_parameter<std::string>("port", "/dev/ttyACM0");
        this->declare_parameter<int>("baudrate", 115200);
        this->declare_parameter<int>("send_frequency", 500);
        
        std::string port = this->get_parameter("port").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();
        int send_freq = this->get_parameter("send_frequency").as_int();
        
        try {
            serial_port_.open(port);
            serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
            RCLCPP_INFO(this->get_logger(), "✓ 串口打开: %s @ %d", port.c_str(), baudrate);
        } catch (const boost::system::system_error& e) {
            RCLCPP_ERROR(this->get_logger(), "✗ 串口失败: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&SerialCommNode::cmdVelCallback, this, std::placeholders::_1));

        int period_ms = 1000 / send_freq;
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&SerialCommNode::sendFrame, this));
        
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "串口通信节点初始化完成");
        RCLCPP_INFO(this->get_logger(), "========================================");
    }

    ~SerialCommNode()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            vx_ = vy_ = vyaw_ = 0;
        }
        sendFrame();
        if (serial_port_.is_open()) serial_port_.close();
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        vx_   =  -msg->linear.y / 10.0f;  
        vy_   =  msg->linear.x / 10.0f;  
        vyaw_ =  msg->angular.z / 2.0f;
    }

    void sendFrame()
    {
        NucControlFrame frame;
        frame.SOF = 0x55;
        frame.ID = 0x04;
        
        {
            std::lock_guard<std::mutex> lock(mutex_);
            frame.vx = vx_;
            frame.vy = vy_;
            frame.vyaw = vyaw_;
        }
        
        frame._EOF = 0xFF;
        
        try {
            boost::asio::write(serial_port_, boost::asio::buffer(&frame, sizeof(frame)));
            
            send_count_++;
            auto now = this->now();
            if ((now - last_log_).seconds() >= 1.0) {
                double freq = send_count_ / (now - last_log_).seconds();
                RCLCPP_INFO(this->get_logger(),
                    "发送: %.1f Hz | 车体系: vx=%.3f(右) vy=%.3f(前) vyaw=%.3f",
                    freq, frame.vx, frame.vy, frame.vyaw);
                send_count_ = 0;
                last_log_ = now;
            }
        } catch (const boost::system::system_error& e) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "写入失败");
        }
    }

private:
    boost::asio::io_context io_context_;
    boost::asio::serial_port serial_port_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::mutex mutex_;
    float vx_ = 0, vy_ = 0, vyaw_ = 0;
    size_t send_count_ = 0;
    rclcpp::Time last_log_{this->now()};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialCommNode>());
    rclcpp::shutdown();
    return 0;
}
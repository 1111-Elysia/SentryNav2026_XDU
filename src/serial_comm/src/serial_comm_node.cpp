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
    SerialCommNode() 
        : Node("serial_comm_node"), 
          io_context_(), 
          serial_port_(io_context_)
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
            serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
            serial_port_.set_option(boost::asio::serial_port_base::parity(
                boost::asio::serial_port_base::parity::none));
            serial_port_.set_option(boost::asio::serial_port_base::stop_bits(
                boost::asio::serial_port_base::stop_bits::one));
            serial_port_.set_option(boost::asio::serial_port_base::flow_control(
                boost::asio::serial_port_base::flow_control::none));
            
            RCLCPP_INFO(this->get_logger(), "✓ 串口已打开: %s @ %d", port.c_str(), baudrate);
        } catch (const boost::system::system_error& e) {
            RCLCPP_ERROR(this->get_logger(), "✗ 串口打开失败 %s: %s", port.c_str(), e.what());
            rclcpp::shutdown();
            return;
        }

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&SerialCommNode::cmdVelCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "✓ 已订阅: /cmd_vel");

        int period_ms = 1000 / send_freq;
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&SerialCommNode::sendControlFrame, this));
        
        RCLCPP_INFO(this->get_logger(), "====================================");
        RCLCPP_INFO(this->get_logger(), "串口通信节点初始化完成");
        RCLCPP_INFO(this->get_logger(), "  发送频率: %d Hz", send_freq);
        RCLCPP_WARN(this->get_logger(), "坐标系转换:");
        RCLCPP_WARN(this->get_logger(), "  Nav2输出(ROS系): vx=前进, vy=左移");
        RCLCPP_WARN(this->get_logger(), "  底盘期望(车体系): vx=右移, vy=前进");
        RCLCPP_WARN(this->get_logger(), "  转换: vx_chassis=-vy_ros, vy_chassis=vx_ros");
        RCLCPP_INFO(this->get_logger(), "====================================");
    }

    ~SerialCommNode()
    {
        RCLCPP_INFO(this->get_logger(), "发送停止命令...");
        {
            std::lock_guard<std::mutex> lock(mutex_);
            current_vx_ = 0.0f;
            current_vy_ = 0.0f;
            current_vyaw_ = 0.0f;
        }
        sendControlFrame();
        
        if (serial_port_.is_open()) {
            serial_port_.close();
            RCLCPP_INFO(this->get_logger(), "✓ 串口已关闭");
        }
    }

private:
    // ===== /cmd_vel 回调：ROS坐标系 → 车体坐标系 =====
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Nav2 输出（ROS坐标系）
        float vx_ros = static_cast<float>(msg->linear.x);   // 前进速度
        float vy_ros = static_cast<float>(msg->linear.y);   // 左移速度
        float vyaw_ros = static_cast<float>(msg->angular.z); // 旋转速度
        
        // 转换到车体坐标系（X=右，Y=前）
        // ROS系 → 车体系: 绕Z轴旋转+90度（逆转换）
        // vx_chassis = -vy_ros  (右移 = -左移)
        // vy_chassis = vx_ros   (前进 = 前进)
        current_vx_ = -vy_ros / 10.0;   // 车体X（右移）
        current_vy_ = vx_ros / 10.0;    // 车体Y（前进）
        current_vyaw_ = vyaw_ros / 25.0; // yaw不变
        
        // 周期性日志
        auto now = this->now();
        if ((now - last_log_time_).seconds() > 0.5) {
            RCLCPP_DEBUG(this->get_logger(), 
                "ROS系 [%.2f前, %.2f左] → 车体系 [%.2f右, %.2f前]",
                vx_ros, vy_ros, current_vx_, current_vy_);
            last_log_time_ = now;
        }
    }

    // ===== 发送控制帧 =====
    void sendControlFrame()
    {
        NucControlFrame frame;
        frame.SOF = 0x55;
        frame.ID = 0x04;
        
        {
            std::lock_guard<std::mutex> lock(mutex_);
            frame.vx = current_vx_;    // 车体X（右移）
            frame.vy = current_vy_;    // 车体Y（前进）
            frame.vyaw = current_vyaw_;
        }
        
        frame._EOF = 0xFF;
        
        try {
            size_t bytes_written = boost::asio::write(
                serial_port_, 
                boost::asio::buffer(&frame, sizeof(NucControlFrame))
            );
            
            if (bytes_written != sizeof(NucControlFrame)) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "写入不完整: %zu/%zu bytes", bytes_written, sizeof(NucControlFrame));
            }
            
            send_count_++;
            
            auto now = this->now();
            if ((now - last_stats_time_).seconds() >= 1.0) {
                double actual_freq = send_count_ / (now - last_stats_time_).seconds();
                RCLCPP_INFO(this->get_logger(),
                    "发送统计: %.1f Hz | 车体系: vx=%.2f(右) vy=%.2f(前) vyaw=%.2f",
                    actual_freq, frame.vx, frame.vy, frame.vyaw);
                send_count_ = 0;
                last_stats_time_ = now;
            }
            
        } catch (const boost::system::system_error& e) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "串口写入失败: %s", e.what());
        }
    }

private:
    boost::asio::io_context io_context_;
    boost::asio::serial_port serial_port_;
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    std::mutex mutex_;
    float current_vx_ = 0.0f;    // 车体X（右移）
    float current_vy_ = 0.0f;    // 车体Y（前进）
    float current_vyaw_ = 0.0f;
    
    size_t send_count_ = 0;
    rclcpp::Time last_log_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_stats_time_{this->now()};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<SerialCommNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::cerr << "异常: " << e.what() << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}
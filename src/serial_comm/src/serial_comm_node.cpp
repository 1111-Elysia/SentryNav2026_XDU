#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <boost/asio.hpp>
#include <cstdint>
#include <mutex>
#include <chrono>

// ===== 串口通信协议定义 =====
#pragma pack(push, 1)
typedef struct {
    uint8_t SOF;      // 帧头: 0x55
    uint8_t ID;       // 数据ID: 0x04
    float vx;         // X方向速度 (m/s) - 前后
    float vy;         // Y方向速度 (m/s) - 左右（全向轮）
    float vyaw;       // Yaw轴角速度 (rad/s) - 旋转
    uint8_t _EOF;     // 帧尾: 0xFF
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
        // ===== 声明参数 =====
        this->declare_parameter<std::string>("port", "/dev/ttyACM0");
        this->declare_parameter<int>("baudrate", 115200);
        this->declare_parameter<int>("send_frequency", 500);  // 发送频率 (Hz)
        
        // ===== 获取参数 =====
        std::string port = this->get_parameter("port").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();
        int send_freq = this->get_parameter("send_frequency").as_int();
        
        // ===== 打开串口 =====
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
            
            RCLCPP_INFO(this->get_logger(), 
                "✓ 串口已打开: %s @ %d baud", port.c_str(), baudrate);
        } catch (const boost::system::system_error& e) {
            RCLCPP_ERROR(this->get_logger(), 
                "✗ 无法打开串口 %s: %s", port.c_str(), e.what());
            rclcpp::shutdown();
            return;
        }

        // ===== 订阅 /cmd_vel 话题 =====
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&SerialCommNode::cmdVelCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "✓ 已订阅话题: /cmd_vel");

        // ===== 创建定时器：以指定频率发送控制帧 =====
        int period_ms = 1000 / send_freq;  // 计算周期 (ms)
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&SerialCommNode::sendControlFrame, this));
        
        RCLCPP_INFO(this->get_logger(), 
            "✓ 控制帧发送频率: %d Hz (周期 %d ms)", send_freq, period_ms);
        
        RCLCPP_INFO(this->get_logger(), "====================================");
        RCLCPP_INFO(this->get_logger(), "串口通信节点初始化完成");
        RCLCPP_INFO(this->get_logger(), "====================================");
        RCLCPP_INFO(this->get_logger(), "协议格式:");
        RCLCPP_INFO(this->get_logger(), "  SOF: 0x55");
        RCLCPP_INFO(this->get_logger(), "  ID:  0x04");
        RCLCPP_INFO(this->get_logger(), "  vx:  float (4 bytes) - 前后速度 (m/s)");
        RCLCPP_INFO(this->get_logger(), "  vy:  float (4 bytes) - 左右速度 (m/s)");
        RCLCPP_INFO(this->get_logger(), "  vyaw: float (4 bytes) - 旋转速度 (rad/s)");
        RCLCPP_INFO(this->get_logger(), "  EOF: 0xFF");
        RCLCPP_INFO(this->get_logger(), "  总长度: %zu bytes", sizeof(NucControlFrame));
        RCLCPP_INFO(this->get_logger(), "====================================");
    }

    ~SerialCommNode()
    {
        // ===== 关闭串口前发送停止命令 =====
        RCLCPP_INFO(this->get_logger(), "正在关闭串口，发送停止命令...");
        {
            std::lock_guard<std::mutex> lock(mutex_);
            current_vx_ = 0.0f;
            current_vy_ = 0.0f;
            current_vyaw_ = 0.0f;
        }
        sendControlFrame();  // 发送一次零速度
        
        if (serial_port_.is_open()) {
            serial_port_.close();
            RCLCPP_INFO(this->get_logger(), "✓ 串口已关闭");
        }
    }

private:
    // ===== /cmd_vel 回调函数：更新速度指令 =====
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // 提取速度数据
        current_vx_ = static_cast<float>(msg->linear.x) / 100.0;   // 前后速度
        current_vy_ = static_cast<float>(msg->linear.y) / 100.0;   // 左右速度
        current_vyaw_ = static_cast<float>(msg->angular.z) / 100.0; // 旋转速度
        
        // 周期性输出日志（节流到 2Hz）
        auto now = this->now();
        if ((now - last_log_time_).seconds() > 0.5) {
            RCLCPP_DEBUG(this->get_logger(), 
                "收到速度指令: vx=%.3f m/s, vy=%.3f m/s, vyaw=%.3f rad/s",
                current_vx_, current_vy_, current_vyaw_);
            last_log_time_ = now;
        }
    }

    // ===== 定时器回调：发送控制帧到串口 =====
    void sendControlFrame()
    {
        NucControlFrame frame;
        
        // 填充帧头和ID
        frame.SOF = 0x55;
        frame.ID = 0x04;
        
        // 填充速度数据（加锁保护）
        {
            std::lock_guard<std::mutex> lock(mutex_);
            frame.vx = current_vx_;
            frame.vy = current_vy_;
            frame.vyaw = current_vyaw_;
        }
        
        // 填充帧尾
        frame._EOF = 0xFF;
        
        // 发送到串口
        try {
            size_t bytes_written = boost::asio::write(
                serial_port_, 
                boost::asio::buffer(&frame, sizeof(NucControlFrame))
            );
            
            // 验证写入字节数
            if (bytes_written != sizeof(NucControlFrame)) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "串口写入不完整: %zu/%zu bytes", 
                    bytes_written, sizeof(NucControlFrame));
            }
            
            // 统计发送次数
            send_count_++;
            
            // 周期性输出统计信息（每秒一次）
            auto now = this->now();
            if ((now - last_stats_time_).seconds() >= 1.0) {
                double elapsed = (now - last_stats_time_).seconds();
                double actual_freq = send_count_ / elapsed;
                
                RCLCPP_INFO(this->get_logger(),
                    "发送统计: %.1f Hz | vx=%.3f vy=%.3f vyaw=%.3f",
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
    // ===== Boost ASIO 串口对象 =====
    boost::asio::io_context io_context_;
    boost::asio::serial_port serial_port_;
    
    // ===== ROS2 对象 =====
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // ===== 当前速度数据（线程安全） =====
    std::mutex mutex_;
    float current_vx_ = 0.0f;    // X方向速度 (m/s)
    float current_vy_ = 0.0f;    // Y方向速度 (m/s)
    float current_vyaw_ = 0.0f;  // Yaw角速度 (rad/s)
    
    // ===== 统计信息 =====
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
        std::cerr << "节点异常: " << e.what() << std::endl;
    }
    
    rclcpp::shutdown();
    return 0;
}
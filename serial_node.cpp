#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <boost/asio.hpp>
#include <cstdint>
#include <array>
#include <chrono>
#include <mutex>
#include <thread>
#include <iostream>

using namespace std;

// 协议定义
#pragma pack(push, 1)
typedef struct {
    uint8_t SOF;           // 0x55
    uint8_t ID;            // 0x04
    // uint8_t chassis_mode;  // 底盘模式
    float vx;              // x方向速度 (m/s)
    float vy;              // y方向速度 (m/s)
    float vyaw;          // yaw转动速度 (rad/s)
    // uint32_t sentry_cmd;   // 哨兵自主决策信息
    // uint8_t yaw_flag;
    uint8_t _EOF;           // 0xFF
} NucControlFrame;
#pragma pack(pop)

class SerialCommNode : public rclcpp::Node
{
public:
    SerialCommNode() : Node("serial_comm_node"), io_context_(), serial_port_(io_context_)
    {
        // 串口初始化
        try {
            serial_port_.open("/dev/ttyACM0"); // 根据实际设备修改
            cout<<"1"<<endl;
            serial_port_.set_option(boost::asio::serial_port_base::baud_rate(115200));
            serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
            serial_port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
            serial_port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
            serial_port_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        } catch (const boost::system::system_error& e) {
            RCLCPP_ERROR(this->get_logger(), "无法打开串口: %s", e.what());
            rclcpp::shutdown();
        }

        // 创建cmd_vel订阅者
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&SerialCommNode::cmdVelCallback, this, std::placeholders::_1));

        // 创建云台位姿发布者
        gimbal_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "serial/gimbal_joint_state", 10);

        // 创建500Hz定时器发送控制帧
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2), // 500Hz = 2ms间隔
            std::bind(&SerialCommNode::sendControlFrame, this));

        // 创建定时器读取串口数据
        read_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&SerialCommNode::readSerialData, this));
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // 更新最新速度指令
        std::lock_guard<std::mutex> lock(mutex_);
        current_vx_ = msg->linear.x;
        current_vy_ = msg->linear.y;
        current_yaw_ = msg->angular.z;
    }

    void sendControlFrame()
    {
        NucControlFrame frame;
        frame.SOF = 0x55;
        frame.ID = 0x04;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            // frame.chassis_mode = 1; // 默认为1 (正常模式)
            frame.vx = current_vx_;
            frame.vy = current_vy_;
            frame.vyaw = current_yaw_;
            // frame.sentry_cmd = 0;    // 默认无哨兵指令
        }

        frame._EOF = 0xFF;

        try {
            boost::asio::write(serial_port_, boost::asio::buffer(&frame, sizeof(NucControlFrame)));
        } catch (const boost::system::system_error& e) {
            RCLCPP_ERROR(this->get_logger(), "串口写入失败: %s", e.what());
        }
    }

    void readSerialData()
    {
        try {
            char buffer[256];
            size_t bytes_read = serial_port_.read_some(boost::asio::buffer(buffer));
            if (bytes_read > 0) {
                std::string data(buffer, bytes_read);
                processIncomingData(data);
            }
        } catch (const boost::system::system_error& e) {
            RCLCPP_ERROR(this->get_logger(), "串口读取失败: %s", e.what());
        }
    }

    void processIncomingData(const std::string& data)
    {
        float yaw, pitch;
        if (sscanf(data.c_str(), "G%f,%f", &yaw, &pitch) == 2) {
            auto joint_state = sensor_msgs::msg::JointState();
            joint_state.header.stamp = this->now();
            joint_state.name = {"gimbal_yaw", "gimbal_pitch"};
            joint_state.position = {yaw, pitch};
            gimbal_pub_->publish(joint_state);
        }
    }

    boost::asio::io_context io_context_;
    boost::asio::serial_port serial_port_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gimbal_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr read_timer_;

    // 当前控制数据
    std::mutex mutex_;
    float current_vx_ = 0.0f;
    float current_vy_ = 0.0f;
    float current_yaw_ = 0.0f;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialCommNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

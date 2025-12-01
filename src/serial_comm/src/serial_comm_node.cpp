#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <boost/asio.hpp>
#include <cstdint>
#include <mutex>
#include <chrono>
#include <sentry_msgs/msg/vw.hpp>          
#include <sentry_msgs/msg/scan_mode.hpp>
#include <sentry_msgs/msg/match_stage.hpp>   // 新增

#pragma pack(push, 1)
typedef struct {
    uint8_t SOF;
    uint8_t ID;
    float vx;
    float vy;
    float vyaw;
    float vw;             
    bool  scan_mod_type;  
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

        // 声明并读取话题名参数（默认值与原来一致）
        this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
        this->declare_parameter<std::string>("vw_topic", "/vw");
        this->declare_parameter<std::string>("scan_mod_type_topic", "/scan_mod_type");
        this->declare_parameter<std::string>("match_stage_topic", "/match_stage");  // 新增

        std::string port = this->get_parameter("port").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();
        int send_freq = this->get_parameter("send_frequency").as_int();

        // 获取参数化的话题名
        std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
        std::string vw_topic = this->get_parameter("vw_topic").as_string();
        std::string scan_mod_type_topic = this->get_parameter("scan_mod_type_topic").as_string();
        std::string match_stage_topic = this->get_parameter("match_stage_topic").as_string();  // 新增

        try {
            serial_port_.open(port);
            serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
            RCLCPP_INFO(this->get_logger(), "✓ 串口打开: %s @ %d", port.c_str(), baudrate);
        } catch (const boost::system::system_error& e) {
            RCLCPP_ERROR(this->get_logger(), "✗ 串口失败: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        // 使用参数化的话题名创建订阅
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic, 10,
            std::bind(&SerialCommNode::cmdVelCallback, this, std::placeholders::_1));

        vw_sub_ = this->create_subscription<sentry_msgs::msg::Vw>(
            vw_topic, 10,
            [this](const sentry_msgs::msg::Vw::SharedPtr m){
                std::lock_guard<std::mutex> lk(mutex_);
                vw_ = m->vw;
            });

        scan_mod_sub_ = this->create_subscription<sentry_msgs::msg::ScanMode>(
            scan_mod_type_topic, 10,
            [this](const sentry_msgs::msg::ScanMode::SharedPtr m){
                std::lock_guard<std::mutex> lk(mutex_);
                scan_mod_type_ = m->scan_mod_type;
            });

        // 新增：订阅比赛阶段
        match_stage_sub_ = this->create_subscription<sentry_msgs::msg::MatchStage>(
            match_stage_topic, 10,
            [this](const sentry_msgs::msg::MatchStage::SharedPtr m){
                std::lock_guard<std::mutex> lk(mutex_);
                match_stage_ = m->match_stage;
                // if (match_stage_ != 4) {
                //     RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                //         "比赛阶段=%u (!=4)，控制量清零", match_stage_);
                // }
            });

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
            vw_ = 0;                
            scan_mod_type_ = false; 
        }
        sendFrame();
        if (serial_port_.is_open()) serial_port_.close();
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        vx_   =  msg->linear.x;
        vy_   =  msg->linear.y;
        vyaw_ =  msg->angular.z;
        vyaw_ =  0;
    }

    void sendFrame()
    {
        NucControlFrame frame;
        frame.SOF = 0x55;
        frame.ID = 0x04;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            // 当 match_stage != 4 时，所有控制量设置为 0
            // if (match_stage_ != 4) {
            //     frame.vx = 0;
            //     frame.vy = 0;
            //     frame.vyaw = 0;
            //     frame.vw = 0;
            //     frame.scan_mod_type = false;
            // } else {
            //     frame.vx = vx_;
            //     frame.vy = vy_;
            //     frame.vyaw = vyaw_;
            //     frame.vw = vw_;
            //     frame.scan_mod_type = scan_mod_type_;
            // }
            frame.vx = vx_;
            frame.vy = vy_;
            frame.vyaw = vyaw_;
            frame.vw = vw_;
            frame.scan_mod_type = scan_mod_type_;
        }
        frame._EOF = 0xFF;
        try {
            boost::asio::write(serial_port_, boost::asio::buffer(&frame, sizeof(frame)));
            
            send_count_++;
            auto now = this->now();
            if ((now - last_log_).seconds() >= 1.0) {
                double freq = send_count_ / (now - last_log_).seconds();
                RCLCPP_INFO(this->get_logger(),
                    "发送: %.1f Hz | stage=%u vx=%.3f vy=%.3f vyaw=%.3f vw=%.3f scan=%u",
                    freq, match_stage_, frame.vx, frame.vy, frame.vyaw, frame.vw,
                    (unsigned)frame.scan_mod_type);
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
    rclcpp::Subscription<sentry_msgs::msg::Vw>::SharedPtr vw_sub_;          
    rclcpp::Subscription<sentry_msgs::msg::ScanMode>::SharedPtr scan_mod_sub_;
    rclcpp::Subscription<sentry_msgs::msg::MatchStage>::SharedPtr match_stage_sub_;  // 新增
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::mutex mutex_;
    float vx_ = 0, vy_ = 0, vyaw_ = 0;
    float vw_ = 0;              
    bool  scan_mod_type_ = true;
    uint8_t match_stage_ = 0;
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

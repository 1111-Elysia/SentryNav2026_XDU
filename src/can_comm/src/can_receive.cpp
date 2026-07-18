#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>

using namespace std::chrono_literals;

class CanReceiveNode : public rclcpp::Node
{
public:
    CanReceiveNode()
    : Node("can_receive")
    {
        // 参数声明
        this->declare_parameter<std::string>("port", "can0");
        this->declare_parameter<int>("can_id", 0x180);
        this->declare_parameter<std::string>("yaw_topic", "/target/yaw");
        this->declare_parameter<std::string>("pitch_topic", "/target/pitch");
        this->declare_parameter<std::string>("distance_topic", "/target/distance");
        this->declare_parameter<std::string>("diankong_yaw_topic", "/diankong_yaw");
        this->declare_parameter<double>("publish_rate_hz", 50.0);

        // 读取参数
        std::string port = this->get_parameter("port").as_string();
        int can_id = this->get_parameter("can_id").as_int();
        std::string yaw_topic = this->get_parameter("yaw_topic").as_string();
        std::string pitch_topic = this->get_parameter("pitch_topic").as_string();
        std::string distance_topic = this->get_parameter("distance_topic").as_string();
        std::string diankong_yaw_topic = this->get_parameter("diankong_yaw_topic").as_string();
        double publish_rate_hz = this->get_parameter("publish_rate_hz").as_double();
        if (publish_rate_hz <= 1e-6) publish_rate_hz = 50.0;

        // 创建发布者
        yaw_pub_ = this->create_publisher<std_msgs::msg::Float32>(yaw_topic, 10);
        pitch_pub_ = this->create_publisher<std_msgs::msg::Float32>(pitch_topic, 10);
        distance_pub_ = this->create_publisher<std_msgs::msg::Float32>(distance_topic, 10);
        diankong_yaw_pub_ = this->create_publisher<std_msgs::msg::Float32>(diankong_yaw_topic, 10);

        // 打开 SocketCAN
        socket_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socket_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "SocketCAN: 创建 socket 失败");
            rclcpp::shutdown();
            return;
        }

        // 获取接口索引并绑定
        struct ifreq ifr {};
        std::strcpy(ifr.ifr_name, port.c_str());
        if (::ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
            RCLCPP_ERROR(this->get_logger(), "SocketCAN: 获取 %s 索引失败", port.c_str());
            ::close(socket_fd_);
            socket_fd_ = -1;
            rclcpp::shutdown();
            return;
        }

        struct sockaddr_can addr {};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (::bind(socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "SocketCAN: 绑定 %s 失败", port.c_str());
            ::close(socket_fd_);
            socket_fd_ = -1;
            rclcpp::shutdown();
            return;
        }

        // 设置过滤器，只接收目标 CAN ID
        struct can_filter filter {};
        filter.can_id = static_cast<canid_t>(can_id) & CAN_SFF_MASK;
        filter.can_mask = CAN_SFF_MASK;
        if (::setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0) {
            RCLCPP_WARN(this->get_logger(), "SocketCAN: 设置过滤器失败");
        }

        // 设置为非阻塞模式
        int flags = ::fcntl(socket_fd_, F_GETFL, 0);
        if (flags >= 0) {
            ::fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);
        }

        // 定时发布
        const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&CanReceiveNode::onTimer, this));

        // 初始化 last_log_ 时间为当前时刻，确保时钟类型一致
        last_log_ = this->now();

        RCLCPP_INFO(this->get_logger(), "CAN 接收节点初始化完成 | port=%s can_id=0x%X", port.c_str(), can_id);
    }

    ~CanReceiveNode() override
    {
        if (socket_fd_ >= 0) {
            ::close(socket_fd_);
            socket_fd_ = -1;
        }
    }

private:
    void onTimer()
    {
        if (socket_fd_ < 0) return;

        float yaw = 0.0f, pitch = 0.0f, distance = 0.0f, diankong_yaw = 0.0f;
        bool got_frame = false;

        // 循环读取所有待处理的 CAN 帧，只用最后一帧
        while (true) {
            struct can_frame frame;
            ssize_t nbytes = ::read(socket_fd_, &frame, sizeof(frame));

            if (nbytes <= 0) {
                break;  // 无更多数据
            }
            if (nbytes != CAN_MTU) {
                continue;
            }
            if (frame.can_id & (CAN_ERR_FLAG | CAN_EFF_FLAG)) {
                continue;
            }
            if (frame.can_dlc < 8) {
                continue;
            }

            int16_t yaw_raw = static_cast<int16_t>((static_cast<uint16_t>(frame.data[0]) << 8) |
                                                    static_cast<uint16_t>(frame.data[1]));
            int16_t pitch_raw = static_cast<int16_t>((static_cast<uint16_t>(frame.data[2]) << 8) |
                                                      static_cast<uint16_t>(frame.data[3]));
            int16_t distance_raw = static_cast<int16_t>((static_cast<uint16_t>(frame.data[4]) << 8) |
                                                         static_cast<uint16_t>(frame.data[5]));
            int16_t diankong_yaw_raw = static_cast<int16_t>((static_cast<uint16_t>(frame.data[6]) << 8) |
                                                             static_cast<uint16_t>(frame.data[7]));

            yaw = static_cast<float>(yaw_raw) / 100.0f;
            pitch = static_cast<float>(pitch_raw) / 100.0f;
            distance = static_cast<float>(distance_raw) / 100.0f;
            diankong_yaw = static_cast<float>(diankong_yaw_raw) / 100.0f;
            got_frame = true;
            frame_count_++;
        }

        if (!got_frame) {
            return;
        }

        // 只发布最新一帧
        {
            std_msgs::msg::Float32 msg;
            msg.data = yaw;
            yaw_pub_->publish(msg);
        }
        {
            std_msgs::msg::Float32 msg;
            msg.data = pitch;
            pitch_pub_->publish(msg);
        }
        {
            std_msgs::msg::Float32 msg;
            msg.data = distance;
            distance_pub_->publish(msg);
        }
        {
            std_msgs::msg::Float32 msg;
            msg.data = diankong_yaw;
            diankong_yaw_pub_->publish(msg);
        }

        // 频率日志
        auto now = this->now();
        double dt = (now - last_log_).seconds();
        if (dt >= 1.0) {
            double freq = frame_count_ / dt;
            RCLCPP_INFO(this->get_logger(),
                        "CAN 接收频率: %.1f Hz | yaw=%.2f pitch=%.2f distance=%.2f diankong_yaw=%.2f",
                        freq, yaw, pitch, distance, diankong_yaw);
            frame_count_ = 0;
            last_log_ = now;
        }
    }

private:
    int socket_fd_{-1};

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pitch_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr diankong_yaw_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    size_t frame_count_{0};
    rclcpp::Time last_log_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CanReceiveNode>());
    rclcpp::shutdown();
    return 0;
}

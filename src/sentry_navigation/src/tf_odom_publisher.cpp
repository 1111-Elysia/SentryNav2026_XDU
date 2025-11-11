#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class TfOdomPublisher : public rclcpp::Node
{
public:
    TfOdomPublisher() : Node("tf_odom_publisher"),
                        tf_buffer_(this->get_clock()),
                        tf_listener_(tf_buffer_)
    {
        // ===== 声明参数 =====
        this->declare_parameter<double>("base_link_to_livox_x", 0.1);
        this->declare_parameter<double>("base_link_to_livox_y", 0.0);
        this->declare_parameter<double>("base_link_to_livox_z", 0.0);
        this->declare_parameter<double>("base_link_to_livox_roll", 0.0);
        this->declare_parameter<double>("base_link_to_livox_pitch", 0.0);
        this->declare_parameter<double>("base_link_to_livox_yaw", 0.0);
        this->declare_parameter<double>("publish_rate", 50.0);
        
        // ===== 获取参数 =====
        this->get_parameter("base_link_to_livox_x", livox_offset_x_);
        this->get_parameter("base_link_to_livox_y", livox_offset_y_);
        this->get_parameter("base_link_to_livox_z", livox_offset_z_);
        this->get_parameter("base_link_to_livox_roll", livox_roll_);
        this->get_parameter("base_link_to_livox_pitch", livox_pitch_);
        this->get_parameter("base_link_to_livox_yaw", livox_yaw_);
        
        double publish_rate;
        this->get_parameter("publish_rate", publish_rate);
        
        // ===== 初始化发布器和订阅器 =====
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        
        // 订阅 Fast-LIO (仅用于速度)
        fastlio_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10,
            std::bind(&TfOdomPublisher::fastlioCallback, this, std::placeholders::_1));
        
        // ===== 定时器 =====
        auto period = std::chrono::duration<double>(1.0 / publish_rate);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&TfOdomPublisher::timerCallback, this));
        
        // ===== 发布静态变换 =====
        publishStaticTransform();
        
        RCLCPP_INFO(this->get_logger(), "TF Odom Publisher 初始化完成");
        RCLCPP_INFO(this->get_logger(), "  发布频率: %.1f Hz", publish_rate);
        RCLCPP_INFO(this->get_logger(), "  位置来源: Lightning-LM (map→odom)");
        RCLCPP_INFO(this->get_logger(), "  速度来源: Fast-LIO (/Odometry)");
        RCLCPP_INFO(this->get_logger(), "  /odom 格式: pose=[0,0,0], twist=Fast-LIO");
    }

private:
    // ===== 发布静态变换: base_link → livox_frame =====
    void publishStaticTransform()
    {
        geometry_msgs::msg::TransformStamped static_transform;
        static_transform.header.stamp = this->now();
        static_transform.header.frame_id = "base_link";
        static_transform.child_frame_id = "livox_frame";
        
        static_transform.transform.translation.x = livox_offset_x_;
        static_transform.transform.translation.y = livox_offset_y_;
        static_transform.transform.translation.z = livox_offset_z_;
        
        // 使用 RPY 创建四元数
        tf2::Quaternion q;
        q.setRPY(livox_roll_, livox_pitch_, livox_yaw_);
        static_transform.transform.rotation.x = q.x();
        static_transform.transform.rotation.y = q.y();
        static_transform.transform.rotation.z = q.z();
        static_transform.transform.rotation.w = q.w();

        static_tf_broadcaster_->sendTransform(static_transform);
        
        RCLCPP_INFO(this->get_logger(), 
            "发布静态 TF: base_link → livox_frame");
        RCLCPP_INFO(this->get_logger(),
            "  平移: [%.2f, %.2f, %.2f]",
            livox_offset_x_, livox_offset_y_, livox_offset_z_);
        RCLCPP_INFO(this->get_logger(),
            "  旋转(RPY): [%.2f, %.2f, %.2f] rad",
            livox_roll_, livox_pitch_, livox_yaw_);
    }

    // ===== Fast-LIO 回调：提取速度 =====
    void fastlioCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 保存速度信息（只用 twist，不用 pose）
        latest_twist_ = msg->twist.twist;
        twist_received_ = true;
        
        // 调试输出（节流）
        auto now = this->now();
        if ((now - last_velocity_log_time_).seconds() > 5.0) {
            RCLCPP_DEBUG(this->get_logger(), 
                "Fast-LIO 速度: linear=[%.2f, %.2f, %.2f] angular=[%.2f, %.2f, %.2f]",
                latest_twist_.linear.x, latest_twist_.linear.y, latest_twist_.linear.z,
                latest_twist_.angular.x, latest_twist_.angular.y, latest_twist_.angular.z);
            last_velocity_log_time_ = now;
        }
    }

    // ===== 定时器回调：发布 TF 和 /odom =====
    void timerCallback()
    {
        auto now = this->now();
        
        try {
            // ===== 1. 检查 Lightning-LM 是否发布 map→odom =====
            geometry_msgs::msg::TransformStamped map_to_odom;
            bool has_relocalized = false;
            
            try {
                map_to_odom = tf_buffer_.lookupTransform(
                    "map", "odom", tf2::TimePointZero);
                
                double tx = map_to_odom.transform.translation.x;
                double ty = map_to_odom.transform.translation.y;
                
                // 检测是否有有效的重定位（不是初始的零值）
                if (std::abs(tx) > 0.01 || std::abs(ty) > 0.01) {
                    has_relocalized = true;
                    
                    if (!is_relocalized_) {
                        is_relocalized_ = true;
                        RCLCPP_INFO(this->get_logger(), 
                            "✓ Lightning-LM 重定位成功!");
                        RCLCPP_INFO(this->get_logger(), 
                            "  map→odom: [%.2f, %.2f]", tx, ty);
                    }
                }
            } catch (const tf2::TransformException& ex) {
                // Lightning-LM 还没发布 map→odom
                if (!warning_printed_ && (now - start_time_).seconds() > 5.0) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                        "等待 Lightning-LM 发布 map→odom...");
                    warning_printed_ = true;
                }
            }
            
            // ===== 2. 发布 odom→base_link (单位变换) =====
            geometry_msgs::msg::TransformStamped odom_to_baselink;
            odom_to_baselink.header.stamp = now;
            odom_to_baselink.header.frame_id = "odom";
            odom_to_baselink.child_frame_id = "base_link";
            
            // 单位变换: base_link 固定在 odom 原点
            odom_to_baselink.transform.translation.x = 0.0;
            odom_to_baselink.transform.translation.y = 0.0;
            odom_to_baselink.transform.translation.z = 0.0;
            odom_to_baselink.transform.rotation.x = 0.0;
            odom_to_baselink.transform.rotation.y = 0.0;
            odom_to_baselink.transform.rotation.z = 0.0;
            odom_to_baselink.transform.rotation.w = 1.0;
            
            tf_broadcaster_->sendTransform(odom_to_baselink);
            
            // ===== 3. 发布 /odom 话题 =====
            nav_msgs::msg::Odometry odom_msg;
            odom_msg.header.stamp = now;
            odom_msg.header.frame_id = "odom";
            odom_msg.child_frame_id = "base_link";
            
            // pose: 固定在 odom 原点（与 TF 一致）
            odom_msg.pose.pose.position.x = 0.0;
            odom_msg.pose.pose.position.y = 0.0;
            odom_msg.pose.pose.position.z = 0.0;
            odom_msg.pose.pose.orientation.x = 0.0;
            odom_msg.pose.pose.orientation.y = 0.0;
            odom_msg.pose.pose.orientation.z = 0.0;
            odom_msg.pose.pose.orientation.w = 1.0;
            
            // twist: 来自 Fast-LIO
            if (twist_received_) {
                odom_msg.twist.twist = latest_twist_;
            } else {
                // 如果还没收到 Fast-LIO，速度设为 0
                odom_msg.twist.twist.linear.x = 0.0;
                odom_msg.twist.twist.linear.y = 0.0;
                odom_msg.twist.twist.linear.z = 0.0;
                odom_msg.twist.twist.angular.x = 0.0;
                odom_msg.twist.twist.angular.y = 0.0;
                odom_msg.twist.twist.angular.z = 0.0;
            }
            
            // 协方差
            // 位置协方差：低（因为有 Lightning-LM 重定位）
            odom_msg.pose.covariance[0] = 0.01;   // x
            odom_msg.pose.covariance[7] = 0.01;   // y
            odom_msg.pose.covariance[14] = 0.01;  // z
            odom_msg.pose.covariance[21] = 0.01;  // roll
            odom_msg.pose.covariance[28] = 0.01;  // pitch
            odom_msg.pose.covariance[35] = 0.01;  // yaw
            
            // 速度协方差：取决于 Fast-LIO
            odom_msg.twist.covariance[0] = 0.05;   // vx
            odom_msg.twist.covariance[7] = 0.05;   // vy
            odom_msg.twist.covariance[14] = 0.05;  // vz
            odom_msg.twist.covariance[21] = 0.05;  // vroll
            odom_msg.twist.covariance[28] = 0.05;  // vpitch
            odom_msg.twist.covariance[35] = 0.05;  // vyaw
            
            odom_publisher_->publish(odom_msg);
            
            // ===== 4. 周期性状态输出 =====
            if (has_relocalized && (now - last_status_log_time_).seconds() > 10.0) {
                RCLCPP_INFO(this->get_logger(),
                    "状态: Lightning-LM✓ | Fast-LIO速度%s | "
                    "map→odom=[%.2f, %.2f] | /odom.pose=[0,0,0]",
                    twist_received_ ? "✓" : "✗",
                    map_to_odom.transform.translation.x,
                    map_to_odom.transform.translation.y);
                last_status_log_time_ = now;
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "定时器回调异常: %s", e.what());
        }
    }

private:
    // ===== TF 相关 =====
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // ===== 发布器和订阅器 =====
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fastlio_subscriber_;
    
    // ===== 定时器 =====
    rclcpp::TimerBase::SharedPtr timer_;
    
    // ===== 状态变量 =====
    bool is_relocalized_ = false;
    bool warning_printed_ = false;
    bool twist_received_ = false;
    
    rclcpp::Time start_time_{this->now()};
    rclcpp::Time last_status_log_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_velocity_log_time_{0, 0, RCL_ROS_TIME};
    
    // ===== 速度数据 =====
    geometry_msgs::msg::Twist latest_twist_;
    
    // ===== Livox 偏移 =====
    double livox_offset_x_ = 0.117;
    double livox_offset_y_ = 0.0;
    double livox_offset_z_ = 0.0;
    double livox_roll_ = 0.0;
    double livox_pitch_ = 0.0;
    double livox_yaw_ = 0.0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TfOdomPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
        this->declare_parameter<double>("publish_rate", 50.0);
        
        // ===== 新增：坐标系校正参数 =====
        this->declare_parameter<bool>("use_correction", true);
        this->declare_parameter<double>("correction_x", -13.4);  // = -13 - 0.4
        this->declare_parameter<double>("correction_y", -11.0);   // = -11 - 0
        this->declare_parameter<double>("correction_z", 0.0);
        
        // ===== 获取参数 =====
        this->get_parameter("base_link_to_livox_x", livox_offset_x_);
        this->get_parameter("base_link_to_livox_y", livox_offset_y_);
        this->get_parameter("base_link_to_livox_z", livox_offset_z_);
        this->get_parameter("use_correction", use_correction_);
        this->get_parameter("correction_x", correction_x_);
        this->get_parameter("correction_y", correction_y_);
        this->get_parameter("correction_z", correction_z_);
        
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
        RCLCPP_INFO(this->get_logger(), "  位置来源: Lightning-LM + 校正偏移");
        RCLCPP_INFO(this->get_logger(), "  速度来源: Fast-LIO");
        
        if (use_correction_) {
            RCLCPP_INFO(this->get_logger(), "  坐标校正: [%.2f, %.2f, %.2f]",
                correction_x_, correction_y_, correction_z_);
        }
    }

private:
    void publishStaticTransform()
    {
        geometry_msgs::msg::TransformStamped static_transform;
        static_transform.header.stamp = this->now();
        static_transform.header.frame_id = "base_link";
        static_transform.child_frame_id = "livox_frame";
        
        static_transform.transform.translation.x = livox_offset_x_;
        static_transform.transform.translation.y = livox_offset_y_;
        static_transform.transform.translation.z = livox_offset_z_;
        static_transform.transform.rotation.w = 1.0;

        static_tf_broadcaster_->sendTransform(static_transform);
        
        RCLCPP_INFO(this->get_logger(), 
            "发布静态 TF: base_link → livox_frame [%.2f, %.2f, %.2f]",
            livox_offset_x_, livox_offset_y_, livox_offset_z_);
    }

    void fastlioCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        latest_twist_ = msg->twist.twist;
        twist_received_ = true;
    }

    void timerCallback()
    {
        auto now = this->now();
        
        try {
            // ===== 1. 获取 Lightning-LM 的 map→odom =====
            geometry_msgs::msg::TransformStamped lm_map_to_odom;
            bool has_lm_transform = false;
            
            try {
                lm_map_to_odom = tf_buffer_.lookupTransform(
                    "map", "odom", tf2::TimePointZero);
                has_lm_transform = true;
                
                if (!is_relocalized_) {
                    is_relocalized_ = true;
                    RCLCPP_INFO(this->get_logger(), 
                        "✓ 检测到 Lightning-LM 重定位");
                    RCLCPP_INFO(this->get_logger(), 
                        "  原始位置: [%.2f, %.2f]",
                        lm_map_to_odom.transform.translation.x,
                        lm_map_to_odom.transform.translation.y);
                }
                
            } catch (const tf2::TransformException& ex) {
                if (!warning_printed_ && (now - start_time_).seconds() > 5.0) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                        "等待 Lightning-LM 重定位...");
                    warning_printed_ = true;
                }
            }
            
            // ===== 2. 应用坐标校正（如果启用）=====
            geometry_msgs::msg::TransformStamped corrected_map_to_odom;
            
            if (has_lm_transform && use_correction_) {
                // 提取 Lightning-LM 的原始位置
                double lm_x = lm_map_to_odom.transform.translation.x;
                double lm_y = lm_map_to_odom.transform.translation.y;
                double lm_z = lm_map_to_odom.transform.translation.z;
                
                // 应用校正
                corrected_map_to_odom.header.stamp = now;
                corrected_map_to_odom.header.frame_id = "map";
                corrected_map_to_odom.child_frame_id = "odom";
                corrected_map_to_odom.transform.translation.x = lm_x + correction_x_;
                corrected_map_to_odom.transform.translation.y = lm_y + correction_y_;
                corrected_map_to_odom.transform.translation.z = lm_z + correction_z_;
                corrected_map_to_odom.transform.rotation = lm_map_to_odom.transform.rotation;
                
                // 发布校正后的 map→odom
                tf_broadcaster_->sendTransform(corrected_map_to_odom);
                
                // 第一次校正时输出日志
                if (!correction_logged_) {
                    RCLCPP_INFO(this->get_logger(),
                        "应用坐标校正: [%.2f, %.2f] + [%.2f, %.2f] = [%.2f, %.2f]",
                        lm_x, lm_y,
                        correction_x_, correction_y_,
                        lm_x + correction_x_, lm_y + correction_y_);
                    correction_logged_ = true;
                }
                
            } else if (has_lm_transform) {
                // 不使用校正，直接转发
                corrected_map_to_odom = lm_map_to_odom;
                corrected_map_to_odom.header.stamp = now;
            }
            
            // ===== 3. 发布 odom→base_link (单位变换) =====
            geometry_msgs::msg::TransformStamped odom_to_baselink;
            odom_to_baselink.header.stamp = now;
            odom_to_baselink.header.frame_id = "odom";
            odom_to_baselink.child_frame_id = "base_link";
            odom_to_baselink.transform.translation.x = 0.0;
            odom_to_baselink.transform.translation.y = 0.0;
            odom_to_baselink.transform.translation.z = 0.0;
            odom_to_baselink.transform.rotation.w = 1.0;
            
            tf_broadcaster_->sendTransform(odom_to_baselink);
            
            // ===== 4. 发布 /odom 话题 =====
            nav_msgs::msg::Odometry odom_msg;
            odom_msg.header.stamp = now;
            odom_msg.header.frame_id = "odom";
            odom_msg.child_frame_id = "base_link";
            
            odom_msg.pose.pose.position.x = 0.0;
            odom_msg.pose.pose.position.y = 0.0;
            odom_msg.pose.pose.position.z = 0.0;
            odom_msg.pose.pose.orientation.w = 1.0;
            
            if (twist_received_) {
                odom_msg.twist.twist = latest_twist_;
            }
            
            odom_msg.pose.covariance[0] = 0.01;
            odom_msg.pose.covariance[7] = 0.01;
            odom_msg.pose.covariance[35] = 0.01;
            odom_msg.twist.covariance[0] = 0.05;
            odom_msg.twist.covariance[7] = 0.05;
            odom_msg.twist.covariance[35] = 0.05;
            
            odom_publisher_->publish(odom_msg);
            
            // ===== 5. 周期性状态输出 =====
            if (has_lm_transform && (now - last_status_log_time_).seconds() > 5.0) {
                double final_x = corrected_map_to_odom.transform.translation.x;
                double final_y = corrected_map_to_odom.transform.translation.y;
                
                RCLCPP_INFO(this->get_logger(),
                    "状态: 重定位✓ | 速度: Fast-LIO%s | 校正后位置: [%.2f, %.2f]",
                    twist_received_ ? "✓" : "✗",
                    final_x, final_y);
                last_status_log_time_ = now;
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "定时器回调异常: %s", e.what());
        }
    }

private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fastlio_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    bool is_relocalized_ = false;
    bool warning_printed_ = false;
    bool twist_received_ = false;
    bool correction_logged_ = false;
    
    rclcpp::Time start_time_{this->now()};
    rclcpp::Time last_status_log_time_{0, 0, RCL_ROS_TIME};
    
    geometry_msgs::msg::Twist latest_twist_;
    
    double livox_offset_x_ = 0.1;
    double livox_offset_y_ = 0.0;
    double livox_offset_z_ = 0.0;
    
    // ===== 坐标校正参数 =====
    bool use_correction_ = true;
    double correction_x_ = -13.4;
    double correction_y_ = -11.0;
    double correction_z_ = 0.0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TfOdomPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cmath>

class TfOdomPublisher : public rclcpp::Node
{
public:
    TfOdomPublisher() : Node("tf_odom_publisher"),
                        tf_buffer_(this->get_clock()),
                        tf_listener_(tf_buffer_)
    {
        // ===== 声明参数 =====
        this->declare_parameter<double>("base_link_to_livox_x", 0.117);
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
        
        // 订阅 Fast-LIO (用于速度和姿态)
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
        
        RCLCPP_INFO(this->get_logger(), "============================================");
        RCLCPP_INFO(this->get_logger(), "TF Odom Publisher 初始化完成");
        RCLCPP_INFO(this->get_logger(), "============================================");
        RCLCPP_INFO(this->get_logger(), "  发布频率: %.1f Hz", publish_rate);
        RCLCPP_INFO(this->get_logger(), "  位置来源: Lightning-LM (map→odom)");
        RCLCPP_INFO(this->get_logger(), "  速度来源: Fast-LIO (/Odometry)");
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_WARN(this->get_logger(), "坐标系转换已启用:");
        RCLCPP_WARN(this->get_logger(), "  车体坐标系: X=右, Y=前, Z=上");
        RCLCPP_WARN(this->get_logger(), "  ROS 坐标系: X=前, Y=左, Z=上");
        RCLCPP_WARN(this->get_logger(), "  转换方式: 绕Z轴旋转-90度");
        RCLCPP_WARN(this->get_logger(), "  雷达修正: 额外旋转180度");
        RCLCPP_INFO(this->get_logger(), "============================================");
    }

private:
    // ===== 坐标系转换函数: 车体坐标系 → 标准ROS坐标系 =====
    void transformPoseToROS(double x_in, double y_in, double z_in,
                            double qx_in, double qy_in, double qz_in, double qw_in,
                            double& x_out, double& y_out, double& z_out,
                            double& qx_out, double& qy_out, double& qz_out, double& qw_out)
    {
        // 位置转换: X_ros = Y_yours, Y_ros = -X_yours
        x_out = y_in;   // 前方 = 车体Y
        y_out = -x_in;  // 左侧 = -车体X
        z_out = z_in;   // 上方不变
        
        // 姿态转换: q_out = q_in × q_correction
        tf2::Quaternion q_in(qx_in, qy_in, qz_in, qw_in);
        tf2::Quaternion q_correction;
        q_correction.setRPY(0.0, 0.0, -M_PI/2.0);  // 绕Z轴旋转-90度
        
        tf2::Quaternion q_out = q_in * q_correction;
        q_out.normalize();
        
        qx_out = q_out.x();
        qy_out = q_out.y();
        qz_out = q_out.z();
        qw_out = q_out.w();
    }
    
    void transformVelocityToROS(double vx_in, double vy_in, double vz_in,
                                double wx_in, double wy_in, double wz_in,
                                double& vx_out, double& vy_out, double& vz_out,
                                double& wx_out, double& wy_out, double& wz_out)
    {
        // 线速度转换
        vx_out = vy_in;   // ROS前进 = 车体前进
        vy_out = -vx_in;  // ROS左移 = -车体右移
        vz_out = vz_in;
        
        // 角速度转换
        wx_out = wy_in;
        wy_out = -wx_in;
        wz_out = wz_in;  // yaw不变
    }

    // ===== 发布静态变换: base_link → livox_frame =====
    void publishStaticTransform()
    {
        geometry_msgs::msg::TransformStamped static_transform;
        static_transform.header.stamp = this->now();
        static_transform.header.frame_id = "base_link";
        static_transform.child_frame_id = "livox_frame";
        
        // 转换雷达偏移到ROS坐标系
        double livox_x_ros = livox_offset_y_;   // 前方
        double livox_y_ros = -livox_offset_x_;  // 左侧
        double livox_z_ros = livox_offset_z_;
        
        static_transform.transform.translation.x = livox_x_ros;
        static_transform.transform.translation.y = livox_y_ros;
        static_transform.transform.translation.z = livox_z_ros;
        
        // ===== 修改：雷达姿态需要额外旋转180度 =====
        // 原来: livox_yaw_ - M_PI/2.0
        // 现在: livox_yaw_ - M_PI/2.0 + M_PI = livox_yaw_ + M_PI/2.0
        tf2::Quaternion q_lidar;
        q_lidar.setRPY(livox_roll_, livox_pitch_, livox_yaw_ + M_PI/2.0);
        
        static_transform.transform.rotation.x = q_lidar.x();
        static_transform.transform.rotation.y = q_lidar.y();
        static_transform.transform.rotation.z = q_lidar.z();
        static_transform.transform.rotation.w = q_lidar.w();

        static_tf_broadcaster_->sendTransform(static_transform);
        
        RCLCPP_INFO(this->get_logger(), 
            "静态TF: base_link→livox_frame (ROS系): [%.3f, %.3f, %.3f], yaw=%.2f rad",
            livox_x_ros, livox_y_ros, livox_z_ros, livox_yaw_ + M_PI/2.0);
    }

    // ===== Fast-LIO 回调 =====
    void fastlioCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        latest_pose_x_ = msg->pose.pose.position.x;
        latest_pose_y_ = msg->pose.pose.position.y;
        latest_pose_z_ = msg->pose.pose.position.z;
        latest_pose_qx_ = msg->pose.pose.orientation.x;
        latest_pose_qy_ = msg->pose.pose.orientation.y;
        latest_pose_qz_ = msg->pose.pose.orientation.z;
        latest_pose_qw_ = msg->pose.pose.orientation.w;
        
        latest_twist_vx_ = msg->twist.twist.linear.x;
        latest_twist_vy_ = msg->twist.twist.linear.y;
        latest_twist_vz_ = msg->twist.twist.linear.z;
        latest_twist_wx_ = msg->twist.twist.angular.x;
        latest_twist_wy_ = msg->twist.twist.angular.y;
        latest_twist_wz_ = msg->twist.twist.angular.z;
        
        pose_received_ = true;
        twist_received_ = true;
    }

    // ===== 定时器回调 =====
    void timerCallback()
    {
        auto now = this->now();
        
        try {
            // ===== 1. 检查 Lightning-LM =====
            geometry_msgs::msg::TransformStamped map_to_odom;
            bool has_relocalized = false;
            
            try {
                map_to_odom = tf_buffer_.lookupTransform("map", "odom", tf2::TimePointZero);
                double tx = map_to_odom.transform.translation.x;
                double ty = map_to_odom.transform.translation.y;
                
                if (std::abs(tx) > 0.01 || std::abs(ty) > 0.01) {
                    has_relocalized = true;
                    if (!is_relocalized_) {
                        is_relocalized_ = true;
                        RCLCPP_INFO(this->get_logger(), 
                            "✓ Lightning-LM 重定位成功! map→odom=[%.2f, %.2f]", tx, ty);
                    }
                }
            } catch (const tf2::TransformException& ex) {
                if (!warning_printed_ && (now - start_time_).seconds() > 5.0) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                        "等待 Lightning-LM 发布 map→odom...");
                    warning_printed_ = true;
                }
            }
            
            // ===== 2. 发布 odom→base_link (转换Fast-LIO位姿) =====
            geometry_msgs::msg::TransformStamped odom_to_baselink;
            odom_to_baselink.header.stamp = now;
            odom_to_baselink.header.frame_id = "odom";
            odom_to_baselink.child_frame_id = "base_link";
            
            if (pose_received_) {
                // 转换位姿到ROS坐标系
                double x_ros, y_ros, z_ros, qx_ros, qy_ros, qz_ros, qw_ros;
                transformPoseToROS(
                    latest_pose_x_, latest_pose_y_, latest_pose_z_,
                    latest_pose_qx_, latest_pose_qy_, latest_pose_qz_, latest_pose_qw_,
                    x_ros, y_ros, z_ros, qx_ros, qy_ros, qz_ros, qw_ros
                );
                
                odom_to_baselink.transform.translation.x = x_ros;
                odom_to_baselink.transform.translation.y = y_ros;
                odom_to_baselink.transform.translation.z = z_ros;
                odom_to_baselink.transform.rotation.x = qx_ros;
                odom_to_baselink.transform.rotation.y = qy_ros;
                odom_to_baselink.transform.rotation.z = qz_ros;
                odom_to_baselink.transform.rotation.w = qw_ros;
            } else {
                // 还没收到Fast-LIO，保持原点
                odom_to_baselink.transform.translation.x = 0.0;
                odom_to_baselink.transform.translation.y = 0.0;
                odom_to_baselink.transform.translation.z = 0.0;
                odom_to_baselink.transform.rotation.x = 0.0;
                odom_to_baselink.transform.rotation.y = 0.0;
                odom_to_baselink.transform.rotation.z = 0.0;
                odom_to_baselink.transform.rotation.w = 1.0;
            }
            
            tf_broadcaster_->sendTransform(odom_to_baselink);
            
            // ===== 3. 发布 /odom 话题 =====
            nav_msgs::msg::Odometry odom_msg;
            odom_msg.header.stamp = now;
            odom_msg.header.frame_id = "odom";
            odom_msg.child_frame_id = "base_link";
            
            // pose: 与TF保持一致
            if (pose_received_) {
                double x_ros, y_ros, z_ros, qx_ros, qy_ros, qz_ros, qw_ros;
                transformPoseToROS(
                    latest_pose_x_, latest_pose_y_, latest_pose_z_,
                    latest_pose_qx_, latest_pose_qy_, latest_pose_qz_, latest_pose_qw_,
                    x_ros, y_ros, z_ros, qx_ros, qy_ros, qz_ros, qw_ros
                );
                
                odom_msg.pose.pose.position.x = x_ros;
                odom_msg.pose.pose.position.y = y_ros;
                odom_msg.pose.pose.position.z = z_ros;
                odom_msg.pose.pose.orientation.x = qx_ros;
                odom_msg.pose.pose.orientation.y = qy_ros;
                odom_msg.pose.pose.orientation.z = qz_ros;
                odom_msg.pose.pose.orientation.w = qw_ros;
            } else {
                odom_msg.pose.pose.position.x = 0.0;
                odom_msg.pose.pose.position.y = 0.0;
                odom_msg.pose.pose.position.z = 0.0;
                odom_msg.pose.pose.orientation.w = 1.0;
            }
            
            // twist: 转换速度到ROS坐标系
            if (twist_received_) {
                double vx_ros, vy_ros, vz_ros, wx_ros, wy_ros, wz_ros;
                transformVelocityToROS(
                    latest_twist_vx_, latest_twist_vy_, latest_twist_vz_,
                    latest_twist_wx_, latest_twist_wy_, latest_twist_wz_,
                    vx_ros, vy_ros, vz_ros, wx_ros, wy_ros, wz_ros
                );
                
                odom_msg.twist.twist.linear.x = vx_ros;
                odom_msg.twist.twist.linear.y = vy_ros;
                odom_msg.twist.twist.linear.z = vz_ros;
                odom_msg.twist.twist.angular.x = wx_ros;
                odom_msg.twist.twist.angular.y = wy_ros;
                odom_msg.twist.twist.angular.z = wz_ros;
            }
            
            // 协方差
            odom_msg.pose.covariance[0] = 0.01;
            odom_msg.pose.covariance[7] = 0.01;
            odom_msg.pose.covariance[14] = 0.01;
            odom_msg.pose.covariance[21] = 0.01;
            odom_msg.pose.covariance[28] = 0.01;
            odom_msg.pose.covariance[35] = 0.01;
            
            odom_msg.twist.covariance[0] = 0.05;
            odom_msg.twist.covariance[7] = 0.05;
            odom_msg.twist.covariance[14] = 0.05;
            odom_msg.twist.covariance[21] = 0.05;
            odom_msg.twist.covariance[28] = 0.05;
            odom_msg.twist.covariance[35] = 0.05;
            
            odom_publisher_->publish(odom_msg);
            
            // ===== 4. 周期性日志 =====
            if (has_relocalized && (now - last_status_log_time_).seconds() > 10.0) {
                RCLCPP_INFO(this->get_logger(), "状态: Lightning-LM✓ | Fast-LIO✓ (已转换坐标系)");
                last_status_log_time_ = now;
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "定时器异常: %s", e.what());
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
    bool pose_received_ = false;
    bool twist_received_ = false;
    
    rclcpp::Time start_time_{this->now()};
    rclcpp::Time last_status_log_time_{0, 0, RCL_ROS_TIME};
    
    double latest_pose_x_ = 0.0, latest_pose_y_ = 0.0, latest_pose_z_ = 0.0;
    double latest_pose_qx_ = 0.0, latest_pose_qy_ = 0.0, latest_pose_qz_ = 0.0, latest_pose_qw_ = 1.0;
    double latest_twist_vx_ = 0.0, latest_twist_vy_ = 0.0, latest_twist_vz_ = 0.0;
    double latest_twist_wx_ = 0.0, latest_twist_wy_ = 0.0, latest_twist_wz_ = 0.0;
    
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
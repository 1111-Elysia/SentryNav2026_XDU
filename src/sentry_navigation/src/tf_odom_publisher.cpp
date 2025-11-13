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
        this->declare_parameter<double>("base_link_to_livox_x", 0.117);
        this->declare_parameter<double>("base_link_to_livox_y", 0.0);
        this->declare_parameter<double>("base_link_to_livox_z", 0.0);
        this->declare_parameter<double>("publish_rate", 50.0);
        
        this->get_parameter("base_link_to_livox_x", livox_offset_x_);
        this->get_parameter("base_link_to_livox_y", livox_offset_y_);
        this->get_parameter("base_link_to_livox_z", livox_offset_z_);
        double publish_rate;
        this->get_parameter("publish_rate", publish_rate);
        
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        
        fastlio_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10,
            std::bind(&TfOdomPublisher::fastlioCallback, this, std::placeholders::_1));
        
        auto period = std::chrono::duration<double>(1.0 / publish_rate);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&TfOdomPublisher::timerCallback, this));
        
        publishStaticTransform();
        
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "TF Odom Publisher 初始化完成");
        RCLCPP_INFO(this->get_logger(), "  车体系: X=右, Y=前, Z=上");
        RCLCPP_INFO(this->get_logger(), "  ROS系: X=前, Y=左, Z=上");
        RCLCPP_INFO(this->get_logger(), "  转换: 绕Z轴旋转-90度");
        RCLCPP_INFO(this->get_logger(), "========================================");
    }

private:
    void vehicleToROS(double x_v, double y_v, double z_v,
                      double qx_v, double qy_v, double qz_v, double qw_v,
                      double& x_r, double& y_r, double& z_r,
                      double& qx_r, double& qy_r, double& qz_r, double& qw_r)
    {
        x_r = y_v;
        y_r = -x_v;
        z_r = z_v;
        
        tf2::Quaternion q_v(qx_v, qy_v, qz_v, qw_v);
        tf2::Quaternion q_rot;
        q_rot.setRPY(0, 0, -M_PI/2.0);
        tf2::Quaternion q_r = q_v * q_rot;
        q_r.normalize();
        
        qx_r = q_r.x();
        qy_r = q_r.y();
        qz_r = q_r.z();
        qw_r = q_r.w();
    }
    
    void vehicleVelToROS(double vx_v, double vy_v, double vz_v,
                         double wx_v, double wy_v, double wz_v,
                         double& vx_r, double& vy_r, double& vz_r,
                         double& wx_r, double& wy_r, double& wz_r)
    {
        vx_r = vy_v;
        vy_r = -vx_v;
        vz_r = vz_v;
        wx_r = wy_v;
        wy_r = -wx_v;
        wz_r = wz_v;
    }

    void publishStaticTransform()
    {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = this->now();
        tf.header.frame_id = "base_link";
        tf.child_frame_id = "livox_frame";

        // 车体系: x=右, y=前, z=上 → 雷达在前方0.117m 
        tf.transform.translation.x = 0.117;     // 右
        tf.transform.translation.y = 0.0;   // 前
        tf.transform.translation.z = 0.0;     // 上

        // 顺时针90度
        tf2::Quaternion q;
        q.setRPY(0, 0, M_PI/2.0);
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();

        static_tf_broadcaster_->sendTransform(tf);
        RCLCPP_INFO(this->get_logger(), "静态TF base_link->livox_frame: (x=0.0, y=0.117), yaw=-90deg");
    }

    void fastlioCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        pose_x_ = msg->pose.pose.position.x;
        pose_y_ = msg->pose.pose.position.y;
        pose_z_ = msg->pose.pose.position.z;
        pose_qx_ = msg->pose.pose.orientation.x;
        pose_qy_ = msg->pose.pose.orientation.y;
        pose_qz_ = msg->pose.pose.orientation.z;
        pose_qw_ = msg->pose.pose.orientation.w;
        
        vel_vx_ = msg->twist.twist.linear.x;
        vel_vy_ = msg->twist.twist.linear.y;
        vel_vz_ = msg->twist.twist.linear.z;
        vel_wx_ = msg->twist.twist.angular.x;
        vel_wy_ = msg->twist.twist.angular.y;
        vel_wz_ = msg->twist.twist.angular.z;
        
        last_fastlio_stamp_ = msg->header.stamp;  // 记录时间戳

        has_fastlio_ = true;
    }

    void timerCallback()
    {
        auto now = this->now();

        geometry_msgs::msg::TransformStamped tf_odom;
        tf_odom.header.stamp = last_fastlio_stamp_;
        tf_odom.header.frame_id = "odom";
        tf_odom.child_frame_id = "base_link";

        if (has_fastlio_) {
            double x,y,z,qx,qy,qz,qw;
            vehicleToROS(pose_x_, pose_y_, pose_z_,
                         pose_qx_, pose_qy_, pose_qz_, pose_qw_,
                         x, y, z, qx, qy, qz, qw);
            tf_odom.transform.translation.x = x;
            tf_odom.transform.translation.y = y;
            tf_odom.transform.translation.z = z;
            tf_odom.transform.rotation.x = qx;
            tf_odom.transform.rotation.y = qy;
            tf_odom.transform.rotation.z = qz;
            tf_odom.transform.rotation.w = qw;
        } else {
            tf_odom.transform.translation.x = 0.0;
            tf_odom.transform.translation.y = 0.0;
            tf_odom.transform.translation.z = 0.0;
            tf_odom.transform.rotation.x = 0.0;
            tf_odom.transform.rotation.y = 0.0;
            tf_odom.transform.rotation.z = 0.0;
            tf_odom.transform.rotation.w = 1.0;
        }

        tf_broadcaster_->sendTransform(tf_odom);

        // /odom 话题保持 pose=0，twist=Fast-LIO(ROS系)
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = tf_odom.header.stamp;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = 0.0;
        odom_msg.pose.pose.position.y = 0.0;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = 0.0;
        odom_msg.pose.pose.orientation.w = 1.0;

        if (has_fastlio_) {
            double vx, vy, wz;
            vx = vel_vy_;        // 车体→ROS：vx=vy_v
            vy = -vel_vx_;       //          vy=-vx_v
            wz = vel_wz_;
            odom_msg.twist.twist.linear.x  = vx;
            odom_msg.twist.twist.linear.y  = vy;
            odom_msg.twist.twist.angular.z = wz;
        }

        odom_msg.pose.covariance[0] = 0.01;
        odom_msg.pose.covariance[7] = 0.01;
        odom_msg.pose.covariance[35] = 0.01;
        odom_msg.twist.covariance[0] = 0.05;
        odom_msg.twist.covariance[7] = 0.05;
        odom_msg.twist.covariance[35] = 0.05;

        odom_publisher_->publish(odom_msg);

    }

private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fastlio_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_fastlio_stamp_;
    
    bool is_relocalized_ = false;
    bool warning_printed_ = false;
    bool has_fastlio_ = false;
    
    rclcpp::Time start_time_{this->now()};
    rclcpp::Time last_log_{0, 0, RCL_ROS_TIME};
    
    double pose_x_=0, pose_y_=0, pose_z_=0;
    double pose_qx_=0, pose_qy_=0, pose_qz_=0, pose_qw_=1;
    double vel_vx_=0, vel_vy_=0, vel_vz_=0;
    double vel_wx_=0, vel_wy_=0, vel_wz_=0;
    
    double livox_offset_x_ = 0.117;
    double livox_offset_y_ = 0.0;
    double livox_offset_z_ = 0.0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfOdomPublisher>());
    rclcpp::shutdown();
    return 0;
}
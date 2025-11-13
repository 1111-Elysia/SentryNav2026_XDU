#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cmath>
#include <chrono>

class TfOdomPublisher : public rclcpp::Node
{
public:
    TfOdomPublisher() : Node("tf_odom_publisher"),
                        tf_buffer_(this->get_clock()),
                        tf_listener_(tf_buffer_)
    {
        // 参数：雷达外参与发布频率
        this->declare_parameter<double>("base_link_to_livox_x", 0.0);
        this->declare_parameter<double>("base_link_to_livox_y", 0.117);
        this->declare_parameter<double>("base_link_to_livox_z", 0.0);
        this->declare_parameter<double>("livox_yaw_deg", -90.0); // 顺时针90°
        this->declare_parameter<double>("publish_rate", 50.0);

        this->get_parameter("base_link_to_livox_x", livox_offset_x_);
        this->get_parameter("base_link_to_livox_y", livox_offset_y_);
        this->get_parameter("base_link_to_livox_z", livox_offset_z_);
        this->get_parameter("livox_yaw_deg", livox_yaw_deg_);
        double publish_rate;
        this->get_parameter("publish_rate", publish_rate);

        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        // 订阅 FAST_LIO（它发布 odom->base_link TF，本节点不再重复发布）
        fastlio_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 50,
            std::bind(&TfOdomPublisher::fastlioCallback, this, std::placeholders::_1));

        // 周期发布 /odom（pose=0，twist=FAST_LIO 转换后）
        auto period = std::chrono::duration<double>(1.0 / publish_rate);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&TfOdomPublisher::timerCallback, this));

        publishStaticTransform();

        RCLCPP_INFO(this->get_logger(), "tf_odom_publisher: 仅发布 base_link->livox_frame 静态TF；/odom 仅速度。");
    }

private:
    // 车体系(右/前/上) → ROS(前/左/上) 的速度映射
    inline void vehicleVelToROS(double vx_v, double vy_v, double vz_v,
                                double wx_v, double wy_v, double wz_v,
                                double& vx_r, double& vy_r, double& vz_r,
                                double& wx_r, double& wy_r, double& wz_r)
    {
        vx_r = vy_v;      // 前 = 车体前
        vy_r = -vx_v;     // 左 = -车体右
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

        tf.transform.translation.x = livox_offset_x_; // 车体系: x=右
        tf.transform.translation.y = livox_offset_y_; // 车体系: y=前
        tf.transform.translation.z = livox_offset_z_; // 车体系: z=上

        // 顺时针90°(默认)；可用参数 livox_yaw_deg 改
        tf2::Quaternion q;
        double yaw_rad = livox_yaw_deg_ * M_PI / 180.0;
        q.setRPY(0, 0, yaw_rad);
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();

        static_tf_broadcaster_->sendTransform(tf);
        RCLCPP_INFO(this->get_logger(),
            "静态TF base_link->livox_frame: t=(%.3f, %.3f, %.3f), yaw=%.1f deg",
            livox_offset_x_, livox_offset_y_, livox_offset_z_, livox_yaw_deg_);
    }

    void fastlioCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // FAST_LIO 速度在车体系(右/前/上)
        vel_vx_ = msg->twist.twist.linear.x;
        vel_vy_ = msg->twist.twist.linear.y;
        vel_vz_ = msg->twist.twist.linear.z;
        vel_wx_ = msg->twist.twist.angular.x;
        vel_wy_ = msg->twist.twist.angular.y;
        vel_wz_ = msg->twist.twist.angular.z;

        last_fastlio_stamp_ = msg->header.stamp;
        has_fastlio_ = true;
    }

    void timerCallback()
    {
        // 不发布任何 TF（odom->base_link 由 FAST_LIO 提供）

        // /odom：pose=0，仅速度；时间戳对齐 FAST_LIO
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = has_fastlio_ ? last_fastlio_stamp_ : this->now();
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
            double vx, vy, vz, wx, wy, wz;
            vehicleVelToROS(vel_vx_, vel_vy_, vel_vz_,
                            vel_wx_, vel_wy_, vel_wz_,
                            vx, vy, vz, wx, wy, wz);
            odom_msg.twist.twist.linear.x  = vx;
            odom_msg.twist.twist.linear.y  = vy;
            odom_msg.twist.twist.linear.z  = vz;
            odom_msg.twist.twist.angular.x = wx;
            odom_msg.twist.twist.angular.y = wy;
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
    // 仅静态 TF
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fastlio_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time last_fastlio_stamp_;
    bool has_fastlio_ = false;

    // FAST_LIO 速度缓存（车体系）
    double vel_vx_ = 0, vel_vy_ = 0, vel_vz_ = 0;
    double vel_wx_ = 0, vel_wy_ = 0, vel_wz_ = 0;

    // 雷达外参
    double livox_offset_x_ = 0.0;
    double livox_offset_y_ = 0.117;
    double livox_offset_z_ = 0.0;
    double livox_yaw_deg_ = -90.0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfOdomPublisher>());
    rclcpp::shutdown();
    return 0;
}
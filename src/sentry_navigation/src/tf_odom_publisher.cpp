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
#include <tf2/time.h>
#include <cmath>
#include <chrono>

class TfOdomPublisher : public rclcpp::Node
{
public:
    TfOdomPublisher() : Node("tf_odom_publisher"),
                        tf_buffer_(this->get_clock()),
                        tf_listener_(tf_buffer_)
    {
        this->declare_parameter<double>("base_link_to_livox_x", 0.0);
        this->declare_parameter<double>("base_link_to_livox_y", 0.117);
        this->declare_parameter<double>("base_link_to_livox_z", 0.0);
        this->declare_parameter<double>("publish_rate", 50.0);

        this->get_parameter("base_link_to_livox_x", livox_offset_x_);
        this->get_parameter("base_link_to_livox_y", livox_offset_y_);
        this->get_parameter("base_link_to_livox_z", livox_offset_z_);
        double publish_rate = 50.0;
        this->get_parameter("publish_rate", publish_rate);

        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        fastlio_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 50,
            std::bind(&TfOdomPublisher::fastlioCallback, this, std::placeholders::_1));

        auto period = std::chrono::duration<double>(1.0 / publish_rate);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&TfOdomPublisher::timerCallback, this));

        publishStaticTransform();

        RCLCPP_INFO(this->get_logger(),
                    "tf_odom_publisher: 发布 base_link->livox_frame 静态TF、map->odom 动态TF 和 /odom 话题");
    }

private:
    inline void vehicleVelToROS(double vx_v, double vy_v, double vz_v,
                                double wx_v, double wy_v, double wz_v,
                                double &vx_r, double &vy_r, double &vz_r,
                                double &wx_r, double &wy_r, double &wz_r)
    {
        vx_r = vy_v;
        vy_r = -vx_v;
        vz_r = vz_v;
        wx_r = wy_v;
        wy_r = -wx_v;
        wz_r = wz_v;
    }

    geometry_msgs::msg::Pose vehiclePoseToROS(const geometry_msgs::msg::Pose &pose_v)
    {
        geometry_msgs::msg::Pose pose_r;

        pose_r.position.x = pose_v.position.y;
        pose_r.position.y = -pose_v.position.x;
        pose_r.position.z = pose_v.position.z;

        tf2::Quaternion q_vehicle;
        tf2::fromMsg(pose_v.orientation, q_vehicle);

        tf2::Quaternion q_transform;
        q_transform.setRPY(0, 0, -M_PI / 2.0);

        tf2::Quaternion q_ros = q_transform * q_vehicle;
        pose_r.orientation = tf2::toMsg(q_ros);

        return pose_r;
    }

    void publishStaticTransform()
    {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = this->now();
        tf.header.frame_id = "base_link";
        tf.child_frame_id = "livox_frame";

        tf.transform.translation.x = livox_offset_x_;
        tf.transform.translation.y = livox_offset_y_;
        tf.transform.translation.z = livox_offset_z_;
        tf.transform.rotation.x = 0.0;
        tf.transform.rotation.y = 0.0;
        tf.transform.rotation.z = 0.0;
        tf.transform.rotation.w = 1.0;

        static_tf_broadcaster_->sendTransform(tf);
        RCLCPP_INFO(this->get_logger(),
                    "静态TF base_link->livox_frame: t=(%.3f, %.3f, %.3f), 无旋转",
                    livox_offset_x_, livox_offset_y_, livox_offset_z_);
    }

    void fastlioCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        vel_vx_ = msg->twist.twist.linear.x;
        vel_vy_ = msg->twist.twist.linear.y;
        vel_vz_ = msg->twist.twist.linear.z;
        vel_wx_ = msg->twist.twist.angular.x;
        vel_wy_ = msg->twist.twist.angular.y;
        vel_wz_ = msg->twist.twist.angular.z;

        odom_pose_vehicle_ = msg->pose.pose;

        last_fastlio_stamp_ = msg->header.stamp;
        has_fastlio_ = true;
    }

    void timerCallback()
    {
        try
        {
            geometry_msgs::msg::TransformStamped map_to_baselink =
                tf_buffer_.lookupTransform(
                    "map", "base_link",
                    rclcpp::Time(0),
                    rclcpp::Duration::from_seconds(0.05));

            geometry_msgs::msg::TransformStamped odom_to_baselink =
                tf_buffer_.lookupTransform(
                    "odom", "base_link",
                    rclcpp::Time(0),
                    rclcpp::Duration::from_seconds(0.15));

            tf2::Transform tf_map_to_baselink;
            tf2::fromMsg(map_to_baselink.transform, tf_map_to_baselink);

            tf2::Transform tf_odom_to_baselink;
            tf2::fromMsg(odom_to_baselink.transform, tf_odom_to_baselink);

            tf2::Transform tf_map_to_odom = tf_map_to_baselink * tf_odom_to_baselink.inverse();

            geometry_msgs::msg::TransformStamped map_to_odom_msg;
            map_to_odom_msg.header.stamp = map_to_baselink.header.stamp;
            map_to_odom_msg.header.frame_id = "map";
            map_to_odom_msg.child_frame_id = "odom";
            map_to_odom_msg.transform = tf2::toMsg(tf_map_to_odom);

            tf_broadcaster_->sendTransform(map_to_odom_msg);

            if (!has_published_tf_)
            {
                RCLCPP_INFO(this->get_logger(), "成功发布 map->odom TF");
                has_published_tf_ = true;
            }
            has_warned_tf_ = false;
        }
        catch (const tf2::TransformException &ex)
        {
            if (!has_warned_tf_)
            {
                RCLCPP_WARN(this->get_logger(), "无法获取所需TF: %s", ex.what());
                has_warned_tf_ = true;
            }
        }

        if (!has_fastlio_)
        {
            return;
        }

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = last_fastlio_stamp_;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose = vehiclePoseToROS(odom_pose_vehicle_);

        double vx, vy, vz, wx, wy, wz;
        vehicleVelToROS(vel_vx_, vel_vy_, vel_vz_,
                        vel_wx_, vel_wy_, vel_wz_,
                        vx, vy, vz, wx, wy, wz);

        odom_msg.twist.twist.linear.x = vx;
        odom_msg.twist.twist.linear.y = vy;
        odom_msg.twist.twist.linear.z = vz;
        odom_msg.twist.twist.angular.x = wx;
        odom_msg.twist.twist.angular.y = wy;
        odom_msg.twist.twist.angular.z = wz;

        odom_msg.pose.covariance[0] = 0.01;
        odom_msg.pose.covariance[7] = 0.01;
        odom_msg.pose.covariance[35] = 0.01;
        odom_msg.twist.covariance[0] = 0.05;
        odom_msg.twist.covariance[7] = 0.05;
        odom_msg.twist.covariance[35] = 0.05;

        odom_publisher_->publish(odom_msg);
    }

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fastlio_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time last_fastlio_stamp_;
    bool has_fastlio_ = false;
    bool has_warned_tf_ = false;
    bool has_published_tf_ = false;

    double vel_vx_ = 0, vel_vy_ = 0, vel_vz_ = 0;
    double vel_wx_ = 0, vel_wy_ = 0, vel_wz_ = 0;
    geometry_msgs::msg::Pose odom_pose_vehicle_;

    double livox_offset_x_ = 0.0;
    double livox_offset_y_ = 0.117;
    double livox_offset_z_ = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfOdomPublisher>());
    rclcpp::shutdown();
    return 0;
}
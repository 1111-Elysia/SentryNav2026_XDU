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
#include <cmath>
#include <chrono>

class TfOdomPublisher : public rclcpp::Node
{
public:
    TfOdomPublisher()
        : Node("tf_odom_publisher"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_, this, true) // 独立线程处理 /tf，防止阻塞
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
        // 尝试获取最新 TF，失败则回退到缓存
        geometry_msgs::msg::TransformStamped map_to_baselink_msg;
        geometry_msgs::msg::TransformStamped odom_to_baselink_msg;
        rclcpp::Time now = this->now();

        // 新增：标记是否使用了缓存
        bool used_cached_map_bl = false;
        bool used_cached_odom_bl = false;

        // 获取 map->base_link
        bool got_map_bl = false;
        try {
            map_to_baselink_msg = tf_buffer_.lookupTransform("map", "base_link", rclcpp::Time(0));
            last_map_to_baselink_msg_ = map_to_baselink_msg;
            have_map_tf_ = true;
            got_map_bl = true;
        } catch (const tf2::TransformException &ex) {
            if (have_map_tf_) {
                const double age = (now - last_map_to_baselink_msg_.header.stamp).seconds();
                if (age < 0.3) {
                    map_to_baselink_msg = last_map_to_baselink_msg_;
                    used_cached_map_bl = true;   // 标记使用了缓存
                    got_map_bl = true;
                } else {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                         "map->base_link 不可用且缓存过旧(%.2fs): %s", age, ex.what());
                }
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "等待 map->base_link: %s", ex.what());
            }
        }

        // 获取 odom->base_link
        bool got_odom_bl = false;
        try {
            odom_to_baselink_msg = tf_buffer_.lookupTransform("odom", "base_link", rclcpp::Time(0));
            last_odom_to_baselink_msg_ = odom_to_baselink_msg;
            have_odom_tf_ = true;
            got_odom_bl = true;
        } catch (const tf2::TransformException &ex) {
            if (have_odom_tf_) {
                const double age = (now - last_odom_to_baselink_msg_.header.stamp).seconds();
                if (age < 0.5) { // odom只有10Hz，放宽容忍
                    odom_to_baselink_msg = last_odom_to_baselink_msg_;
                    used_cached_odom_bl = true;  // 标记使用了缓存
                    got_odom_bl = true;
                } else {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                         "odom->base_link 不可用且缓存过旧(%.2fs): %s", age, ex.what());
                }
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "等待 odom->base_link: %s", ex.what());
            }
        }

        if (got_map_bl && got_odom_bl)
        {
            try {
                // 计算并发布 map->odom
                tf2::Transform tf_map_to_baselink, tf_odom_to_baselink;
                tf2::fromMsg(map_to_baselink_msg.transform, tf_map_to_baselink);
                tf2::fromMsg(odom_to_baselink_msg.transform, tf_odom_to_baselink);

                tf2::Transform tf_map_to_odom = tf_map_to_baselink * tf_odom_to_baselink.inverse();

                geometry_msgs::msg::TransformStamped map_to_odom_msg;
                // 修复时间类型不匹配：用 if/else 填 builtin_interfaces::msg::Time
                builtin_interfaces::msg::Time stamp;
                if (used_cached_map_bl) {
                    stamp = now;  // rclcpp::Time 可隐式转 builtin_interfaces::msg::Time
                } else {
                    stamp = map_to_baselink_msg.header.stamp;
                }
                map_to_odom_msg.header.stamp = stamp;
                map_to_odom_msg.header.frame_id = "map";
                map_to_odom_msg.child_frame_id = "odom";
                map_to_odom_msg.transform = tf2::toMsg(tf_map_to_odom);

                tf_broadcaster_->sendTransform(map_to_odom_msg);

                if (!has_published_tf_) {
                    RCLCPP_INFO(this->get_logger(), "成功发布 map->odom TF");
                    has_published_tf_ = true;
                }
                has_warned_tf_ = false;
            } catch (const std::exception &e) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "计算/发布 map->odom 失败: %s", e.what());
            }
        }

        // 发布 /odom（保持不变）
        if (!has_fastlio_) return;

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = last_fastlio_stamp_;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose = vehiclePoseToROS(odom_pose_vehicle_);

        double vx, vy, vz, wx, wy, wz;
        vehicleVelToROS(vel_vx_, vel_vy_, vel_vz_, vel_wx_, vel_wy_, vel_wz_, vx, vy, vz, wx, wy, wz);
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

    geometry_msgs::msg::TransformStamped last_map_to_baselink_msg_;
    geometry_msgs::msg::TransformStamped last_odom_to_baselink_msg_;
    bool have_map_tf_ = false;
    bool have_odom_tf_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // 多线程执行器，确保 TF 订阅/Timer/回调都能并行处理
    auto node = std::make_shared<TfOdomPublisher>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <cmath>

class TfOnlyOdom : public rclcpp::Node
{
public:
    TfOnlyOdom()
        : Node("tf_only_odom")
    {
        // 参数：base_link → livox_frame 的变换 C
        this->declare_parameter<double>("base_link_to_livox_x", 0.0);
        this->declare_parameter<double>("base_link_to_livox_y", 0.0);
        this->declare_parameter<double>("base_link_to_livox_z", 0.0);
        this->declare_parameter<double>("base_link_to_livox_roll", 0.0);
        this->declare_parameter<double>("base_link_to_livox_pitch", 0.0);
        this->declare_parameter<double>("base_link_to_livox_yaw", 0.0);
        this->declare_parameter<double>("publish_rate", 50.0);
        this->declare_parameter<bool>("align_base_link_to_map_on_start", true);

        // frame 名称参数（用于 map->odom 静态单位变换）
        this->declare_parameter<std::string>("map_frame_for_A", "map");
        this->declare_parameter<std::string>("odom_frame", "odom");

        // 读取参数
        this->get_parameter("base_link_to_livox_x", livox_offset_x_);
        this->get_parameter("base_link_to_livox_y", livox_offset_y_);
        this->get_parameter("base_link_to_livox_z", livox_offset_z_);

        double roll_deg = 0.0, pitch_deg = 0.0, yaw_deg = 0.0;
        this->get_parameter("base_link_to_livox_roll", roll_deg);
        this->get_parameter("base_link_to_livox_pitch", pitch_deg);
        this->get_parameter("base_link_to_livox_yaw", yaw_deg);

        livox_offset_roll_  = roll_deg * M_PI / 180.0;
        livox_offset_pitch_ = pitch_deg * M_PI / 180.0;
        livox_offset_yaw_   = yaw_deg * M_PI / 180.0;

        double publish_rate = 100.0;
        this->get_parameter("publish_rate", publish_rate);
        this->get_parameter("align_base_link_to_map_on_start", align_base_link_to_map_on_start_);

        this->get_parameter("map_frame_for_A", map_frame_for_A_);
        this->get_parameter("odom_frame", odom_frame_);

        // 计算 C: base_link → livox_frame
        tf_C_.setOrigin(tf2::Vector3(livox_offset_x_, livox_offset_y_, livox_offset_z_));
        tf2::Quaternion q_C;
        q_C.setRPY(livox_offset_roll_, livox_offset_pitch_, livox_offset_yaw_);
        tf_C_.setRotation(q_C);
        tf_C_inv_ = tf_C_.inverse();

        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 50);

        // 订阅 /lio/robo/odom 用于构造 E(odom->livox_frame_two)
        lio_odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/lio/robo/odom",
            rclcpp::QoS(rclcpp::KeepLast(200)).best_effort().durability_volatile(),
            std::bind(&TfOnlyOdom::lioOdomCallback, this, std::placeholders::_1));

        auto period = std::chrono::duration<double>(1.0 / publish_rate);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&TfOnlyOdom::timerCallback, this));

        publishStaticTransforms();

        RCLCPP_INFO(this->get_logger(),
                    "已发布静态 C(base_link->livox_frame) 与静态 A(%s->%s) = 单位变换。",
                    map_frame_for_A_.c_str(), odom_frame_.c_str());
        RCLCPP_INFO(this->get_logger(),
                "align_base_link_to_map_on_start = %s",
                align_base_link_to_map_on_start_ ? "true" : "false");
    }

private:
    void publishStaticTransforms()
    {
        // 静态 C: base_link -> livox_frame
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = rclcpp::Time(0);
        tf_msg.header.frame_id = "base_link";
        tf_msg.child_frame_id = "livox_frame";
        tf_msg.transform = tf2::toMsg(tf_C_);
        static_tf_broadcaster_->sendTransform(tf_msg);
        RCLCPP_INFO(this->get_logger(), "静态变换已发布: base_link -> livox_frame");

        // 静态 A: map_frame_for_A_ -> odom_frame_ 为单位变换 [0,0,0,0,0,0,1]
        geometry_msgs::msg::TransformStamped A_msg;
        A_msg.header.stamp = rclcpp::Time(0);
        A_msg.header.frame_id = map_frame_for_A_;
        A_msg.child_frame_id = odom_frame_;
        A_msg.transform.translation.x = 0.0;
        A_msg.transform.translation.y = 0.0;
        A_msg.transform.translation.z = 0.0;
        A_msg.transform.rotation.x = 0.0;
        A_msg.transform.rotation.y = 0.0;
        A_msg.transform.rotation.z = 0.0;
        A_msg.transform.rotation.w = 1.0;
        static_tf_broadcaster_->sendTransform(A_msg);
        RCLCPP_INFO(this->get_logger(), "静态变换已发布: %s -> %s (单位变换)",
                    map_frame_for_A_.c_str(), odom_frame_.c_str());
    }

    void lioOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!msg) return;
        last_lio_odom_ = *msg;
        have_lio_odom_ = true;
    }

    void timerCallback()
    {
        rclcpp::Time now = this->now();

        // 仅使用 /lio/robo/odom 构造 E(odom->livox_frame_two) → 计算/发布 B 与 /odom
        bool got_E = false;
        if (!have_lio_odom_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "等待 /lio/robo/odom（nav_msgs/Odometry）以构造 E(odom->livox_frame_two)...");
        } else {
            const double age = (now - last_lio_odom_.header.stamp).seconds();
            if (age > 5.0) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "/lio/robo/odom 过旧(%.2fs)，暂不更新 B", age);
            } else {
                got_E = true;
            }
        }

        if (!got_E) return;

        try {
            // 构造 E: odom -> livox_frame_two
            tf2::Transform tf_E;
            tf_E.setOrigin(tf2::Vector3(
                last_lio_odom_.pose.pose.position.x,
                last_lio_odom_.pose.pose.position.y,
                last_lio_odom_.pose.pose.position.z));

            tf2::Quaternion qE(
                last_lio_odom_.pose.pose.orientation.x,
                last_lio_odom_.pose.pose.orientation.y,
                last_lio_odom_.pose.pose.orientation.z,
                last_lio_odom_.pose.pose.orientation.w);
            qE.normalize();
            tf_E.setRotation(qE);

            // 计算 B = E * C_inv
            tf2::Transform tf_B = tf_E * tf_C_inv_;

            // 可选：把首帧 B 作为零点，使启动时 base_link 与 map/odom 对齐。
            tf2::Transform tf_B_out = tf_B;
            if (align_base_link_to_map_on_start_) {
                if (!have_initial_tf_B_) {
                    tf_B0_inv_ = tf_B.inverse();
                    have_initial_tf_B_ = true;
                    RCLCPP_INFO(this->get_logger(),
                                "已记录首帧 odom->base_link，后续发布相对位姿（首帧归零）。");
                }
                tf_B_out = tf_B0_inv_ * tf_B;
            }

            // 发布 B(odom->base_link) TF
            geometry_msgs::msg::TransformStamped B_msg;
            B_msg.header.stamp = last_lio_odom_.header.stamp;
            B_msg.header.frame_id = odom_frame_;
            B_msg.child_frame_id = "base_link";
            B_msg.transform = tf2::toMsg(tf_B_out);
            tf_broadcaster_->sendTransform(B_msg);

            // 发布 /odom（保持原来速度/协方差映射）
            nav_msgs::msg::Odometry odom_msg;
            odom_msg.header.stamp = B_msg.header.stamp;
            odom_msg.header.frame_id = odom_frame_;
            odom_msg.child_frame_id = "base_link";

            odom_msg.pose.pose.position.x = tf_B_out.getOrigin().x();
            odom_msg.pose.pose.position.y = tf_B_out.getOrigin().y();
            odom_msg.pose.pose.position.z = tf_B_out.getOrigin().z();
            odom_msg.pose.pose.orientation = tf2::toMsg(tf_B_out.getRotation());

            tf2::Vector3 linear_vel(
                last_lio_odom_.twist.twist.linear.x,
                last_lio_odom_.twist.twist.linear.y,
                last_lio_odom_.twist.twist.linear.z);
            tf2::Vector3 angular_vel(
                last_lio_odom_.twist.twist.angular.x,
                last_lio_odom_.twist.twist.angular.y,
                last_lio_odom_.twist.twist.angular.z);

            tf2::Vector3 linear_vel_bl = tf_C_inv_.getBasis() * linear_vel;
            tf2::Vector3 angular_vel_bl = tf_C_inv_.getBasis() * angular_vel;

            odom_msg.twist.twist.linear.x = linear_vel_bl.x();
            odom_msg.twist.twist.linear.y = linear_vel_bl.y();
            odom_msg.twist.twist.linear.z = linear_vel_bl.z();
            odom_msg.twist.twist.angular.x = angular_vel_bl.x();
            odom_msg.twist.twist.angular.y = angular_vel_bl.y();
            odom_msg.twist.twist.angular.z = angular_vel_bl.z();

            odom_msg.pose.covariance = last_lio_odom_.pose.covariance;
            odom_msg.twist.covariance = last_lio_odom_.twist.covariance;

            odom_publisher_->publish(odom_msg);

        } catch (const std::exception &e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "计算/发布 B 失败: %s", e.what());
        }
    }

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_odom_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    // C: base_link -> livox_frame
    tf2::Transform tf_C_;
    tf2::Transform tf_C_inv_;
    double livox_offset_x_ = 0.0;
    double livox_offset_y_ = 0.0;
    double livox_offset_z_ = 0.0;
    double livox_offset_roll_ = 0.0;
    double livox_offset_pitch_ = 0.0;
    double livox_offset_yaw_ = 0.0;

    // E 来源为 /lio/robo/odom
    nav_msgs::msg::Odometry last_lio_odom_;
    bool have_lio_odom_ = false;

    bool align_base_link_to_map_on_start_ = true;
    bool have_initial_tf_B_ = false;
    tf2::Transform tf_B0_inv_;

    std::string map_frame_for_A_ = "map";
    std::string odom_frame_ = "odom";
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TfOnlyOdom>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}

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
          tf_listener_(tf_buffer_, this, true) // 独立线程处理 /tf
    {
        // 参数：base_link → livox_frame 的变换 C
        this->declare_parameter<double>("base_link_to_livox_x", 0.0);
        this->declare_parameter<double>("base_link_to_livox_y", 0.117);
        this->declare_parameter<double>("base_link_to_livox_z", 0.0);
        this->declare_parameter<double>("base_link_to_livox_roll", 0.0);
        this->declare_parameter<double>("base_link_to_livox_pitch", 0.0);
        this->declare_parameter<double>("base_link_to_livox_yaw", 0.0);
        this->declare_parameter<double>("publish_rate", 50.0);

        this->get_parameter("base_link_to_livox_x", livox_offset_x_);
        this->get_parameter("base_link_to_livox_y", livox_offset_y_);
        this->get_parameter("base_link_to_livox_z", livox_offset_z_);
        double roll_deg = 0.0, pitch_deg = 0.0, yaw_deg = 0.0;
        this->get_parameter("base_link_to_livox_roll", roll_deg);
        this->get_parameter("base_link_to_livox_pitch", pitch_deg);
        this->get_parameter("base_link_to_livox_yaw", yaw_deg);
        livox_offset_roll_  = roll_deg  * M_PI / 180.0;
        livox_offset_pitch_ = pitch_deg * M_PI / 180.0;
        livox_offset_yaw_   = yaw_deg   * M_PI / 180.0;
        double publish_rate = 50.0;
        this->get_parameter("publish_rate", publish_rate);

        // 计算 C: base_link → livox_frame
        tf_C_.setOrigin(tf2::Vector3(livox_offset_x_, livox_offset_y_, livox_offset_z_));
        tf2::Quaternion q_C;
        q_C.setRPY(livox_offset_roll_, livox_offset_pitch_, livox_offset_yaw_);
        tf_C_.setRotation(q_C);

        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 50);

        // 订阅 FAST-LIO 的 /Odometry（odom→livox_frame_two 的位姿与速度）
        fastlio_odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 50,
            std::bind(&TfOdomPublisher::fastlioOdomCallback, this, std::placeholders::_1));

        auto period = std::chrono::duration<double>(1.0 / publish_rate);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&TfOdomPublisher::timerCallback, this));

        publishStaticTransform();

        RCLCPP_INFO(this->get_logger(),
                    "tf_odom_publisher: 接收 D(map→livox_frame_one) 与 E(odom→livox_frame_two)，发布 A(map→odom)、B(odom→base_link)、C(base_link→livox_frame)");
    }

private:
    void publishStaticTransform()
    {
        // 发布静态 TF: C (base_link → livox_frame)
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = this->now();
        tf_msg.header.frame_id = "base_link";
        tf_msg.child_frame_id = "livox_frame";
        tf_msg.transform = tf2::toMsg(tf_C_);

        static_tf_broadcaster_->sendTransform(tf_msg);
        RCLCPP_INFO(this->get_logger(),
                    "发布静态TF C: base_link→livox_frame t=(%.3f, %.3f, %.3f) rpy=(%.3f, %.3f, %.3f)",
                    livox_offset_x_, livox_offset_y_, livox_offset_z_,
                    livox_offset_roll_, livox_offset_pitch_, livox_offset_yaw_);
    }

    void fastlioOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 缓存 FAST-LIO 的 /Odometry（odom→livox_frame_two 的位姿与速度）
        last_fastlio_odom_ = *msg;
        have_fastlio_odom_ = true;
    }

    void timerCallback()
    {
        rclcpp::Time now = this->now();

        // ===== 获取 D: map → livox_frame_one（重定位包输出，高频） =====
        geometry_msgs::msg::TransformStamped D_msg;
        bool got_D = false;
        bool used_cached_D = false;

        try {
            D_msg = tf_buffer_.lookupTransform("map_one", "livox_frame_one", rclcpp::Time(0));
            last_D_msg_ = D_msg;
            have_D_ = true;
            got_D = true;
        } catch (const tf2::TransformException &ex) {
            if (have_D_) {
                const double age = (now - last_D_msg_.header.stamp).seconds();
                if (age < 0.3) {
                    D_msg = last_D_msg_;
                    used_cached_D = true;
                    got_D = true;
                } else {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                         "D(map→livox_frame_one) 不可用且缓存过旧(%.2fs): %s", age, ex.what());
                }
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "等待 D(map→livox_frame_one): %s", ex.what());
            }
        }

        // ===== 获取 E: odom → livox_frame_two（FAST-LIO 输出，低频 ~10Hz） =====
        geometry_msgs::msg::TransformStamped E_msg;
        bool got_E = false;
        bool used_cached_E = false;

        try {
            E_msg = tf_buffer_.lookupTransform("odom", "livox_frame_two", rclcpp::Time(0));
            last_E_msg_ = E_msg;
            have_E_ = true;
            got_E = true;
        } catch (const tf2::TransformException &ex) {
            if (have_E_) {
                const double age = (now - last_E_msg_.header.stamp).seconds();
                if (age < 0.5) { // 10Hz，放宽容忍
                    E_msg = last_E_msg_;
                    used_cached_E = true;
                    got_E = true;
                } else {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                         "E(odom→livox_frame_two) 不可用且缓存过旧(%.2fs): %s", age, ex.what());
                }
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "等待 E(odom→livox_frame_two): %s", ex.what());
            }
        }

        if (!got_D || !got_E) {
            return; // 等待两者都可用
        }

        try {
            // 转换为 tf2::Transform
            tf2::Transform tf_D, tf_E;
            tf2::fromMsg(D_msg.transform, tf_D);
            tf2::fromMsg(E_msg.transform, tf_E);

            // ===== 计算 A: map → odom = D × E⁻¹ =====
            tf2::Transform tf_A = tf_D * tf_E.inverse();

            geometry_msgs::msg::TransformStamped A_msg;
            // 时间戳：使用 D 的时间戳（高频稳定），若 D 使用缓存则用当前时间
            if (used_cached_D) {
                A_msg.header.stamp = now;
            } else {
                A_msg.header.stamp = D_msg.header.stamp;
            }
            A_msg.header.frame_id = "map";
            A_msg.child_frame_id = "odom";
            A_msg.transform = tf2::toMsg(tf_A);

            tf_broadcaster_->sendTransform(A_msg);

            // ===== 计算 B: odom → base_link = E × C⁻¹ =====
            tf2::Transform tf_B = tf_E * tf_C_.inverse();

            geometry_msgs::msg::TransformStamped B_msg;
            // 时间戳：使用 E 的时间戳（FAST-LIO 里程计时间）
            if (used_cached_E) {
                B_msg.header.stamp = now;
            } else {
                B_msg.header.stamp = E_msg.header.stamp;
            }
            B_msg.header.frame_id = "odom";
            B_msg.child_frame_id = "base_link";
            B_msg.transform = tf2::toMsg(tf_B);

            tf_broadcaster_->sendTransform(B_msg);

            // ===== 发布 /odom 话题（从 FAST-LIO 的 /Odometry 获取，转换坐标系到 odom→base_link） =====
            if (have_fastlio_odom_) {
                nav_msgs::msg::Odometry odom_msg;
                odom_msg.header.stamp = B_msg.header.stamp;
                odom_msg.header.frame_id = "odom";
                odom_msg.child_frame_id = "base_link";

                // 位姿：从 B (odom→base_link) 得到
                odom_msg.pose.pose.position.x = tf_B.getOrigin().x();
                odom_msg.pose.pose.position.y = tf_B.getOrigin().y();
                odom_msg.pose.pose.position.z = tf_B.getOrigin().z();
                odom_msg.pose.pose.orientation = tf2::toMsg(tf_B.getRotation());

                // 速度：直接从 FAST-LIO 的 /Odometry 复制（已经是 odom→livox_frame_two 坐标系的速度）
                // 需要转换到 base_link 坐标系：旋转 C 的逆
                tf2::Vector3 linear_vel(
                    last_fastlio_odom_.twist.twist.linear.x,
                    last_fastlio_odom_.twist.twist.linear.y,
                    last_fastlio_odom_.twist.twist.linear.z
                );
                tf2::Vector3 angular_vel(
                    last_fastlio_odom_.twist.twist.angular.x,
                    last_fastlio_odom_.twist.twist.angular.y,
                    last_fastlio_odom_.twist.twist.angular.z
                );

                // 从 livox_frame_two 坐标系转到 base_link 坐标系：旋转 C⁻¹
                tf2::Vector3 linear_vel_bl = tf_C_.inverse().getBasis() * linear_vel;
                tf2::Vector3 angular_vel_bl = tf_C_.inverse().getBasis() * angular_vel;

                odom_msg.twist.twist.linear.x = linear_vel_bl.x();
                odom_msg.twist.twist.linear.y = linear_vel_bl.y();
                odom_msg.twist.twist.linear.z = linear_vel_bl.z();
                odom_msg.twist.twist.angular.x = angular_vel_bl.x();
                odom_msg.twist.twist.angular.y = angular_vel_bl.y();
                odom_msg.twist.twist.angular.z = angular_vel_bl.z();

                // 协方差：复制 FAST-LIO 的协方差
                odom_msg.pose.covariance = last_fastlio_odom_.pose.covariance;
                odom_msg.twist.covariance = last_fastlio_odom_.twist.covariance;

                odom_publisher_->publish(odom_msg);
            }

            if (!has_published_tf_) {
                RCLCPP_INFO(this->get_logger(), "成功发布 A(map→odom) 与 B(odom→base_link)");
                has_published_tf_ = true;
            }

        } catch (const std::exception &e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "计算/发布 TF 失败: %s", e.what());
        }
    }

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fastlio_odom_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool has_published_tf_ = false;

    // 变换 C: base_link → livox_frame（由参数指定）
    tf2::Transform tf_C_;
    double livox_offset_x_ = 0.0;
    double livox_offset_y_ = 0.0;
    double livox_offset_z_ = 0.0;
    double livox_offset_roll_ = 0.0;
    double livox_offset_pitch_ = 0.0;
    double livox_offset_yaw_ = 0.0;

    // TF 缓存（D 与 E）
    geometry_msgs::msg::TransformStamped last_D_msg_;
    geometry_msgs::msg::TransformStamped last_E_msg_;
    bool have_D_ = false;
    bool have_E_ = false;

    // FAST-LIO /Odometry 缓存
    nav_msgs::msg::Odometry last_fastlio_odom_;
    bool have_fastlio_odom_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // 多线程执行器：确保 TF 订阅/Timer/回调并行处理
    auto node = std::make_shared<TfOdomPublisher>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
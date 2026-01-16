#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using std::placeholders::_1;
using nav_msgs::msg::Odometry;

class OdomPreprocessor : public rclcpp::Node
{
public:
    OdomPreprocessor()
    : Node("odom_preprocessor")
    {
        sub_a_ = create_subscription<Odometry>(
            "/odom_a", 50, std::bind(&OdomPreprocessor::odomA, this, _1));
        sub_b_ = create_subscription<Odometry>(
            "/odom_b", 50, std::bind(&OdomPreprocessor::odomB, this, _1));

        pub_ = create_publisher<Odometry>("/odom_out", 50);

        start_time_ = now();
        RCLCPP_INFO(get_logger(), "odom_preprocessor started");
    }

private:
    /* ======================= 参数 ======================= */
    const double STARTUP_PROTECT_SEC = 3.0;      // 启动保护期
    const double YAW_JUMP_THRESH = 0.35;          // 单步 yaw 跳变
    const double YAW_ACCUM_THRESH = 0.8;          // 累积 yaw 漂移

    /* ======================= 状态 ======================= */
    bool initialized_ = false;
    bool use_backup_ = false;

    double yaw_accum_ = 0.0;

    rclcpp::Time start_time_;

    Odometry last_a_;
    Odometry curr_a_;
    Odometry curr_b_;

    bool have_a_ = false;
    bool have_b_ = false;

    tf2::Transform T_world_b_offset_;

    /* ======================= ROS ======================= */
    rclcpp::Subscription<Odometry>::SharedPtr sub_a_;
    rclcpp::Subscription<Odometry>::SharedPtr sub_b_;
    rclcpp::Publisher<Odometry>::SharedPtr pub_;

    /* ======================= 回调 ======================= */

    void odomA(const Odometry::SharedPtr msg)
    {
        curr_a_ = *msg;
        have_a_ = true;

        if (!initialized_) {
            last_a_ = curr_a_;
            initialized_ = true;
            pub_->publish(curr_a_);
            return;
        }

        if (!use_backup_) {
            if (detectDrift(last_a_, curr_a_)) {
                switchToBackup();
            }
        }

        publish();
        last_a_ = curr_a_;
    }

    void odomB(const Odometry::SharedPtr msg)
    {
        curr_b_ = *msg;
        have_b_ = true;
    }

    /* ======================= 核心逻辑 ======================= */

    bool detectDrift(const Odometry& last, const Odometry& curr)
    {
        double dt = (now() - start_time_).seconds();
        if (dt < STARTUP_PROTECT_SEC)
            return false;

        double yaw_last = getYaw(last.pose.pose.orientation);
        double yaw_curr = getYaw(curr.pose.pose.orientation);
        double dyaw = normalize(yaw_curr - yaw_last);

        yaw_accum_ += std::fabs(dyaw);

        if (std::fabs(dyaw) > YAW_JUMP_THRESH ||
            yaw_accum_ > YAW_ACCUM_THRESH)
        {
            RCLCPP_WARN(get_logger(),
                "Primary odom drift detected! dyaw=%.3f accum=%.3f",
                dyaw, yaw_accum_);
            return true;
        }
        return false;
    }

    void switchToBackup()
    {
        if (!have_b_) {
            RCLCPP_ERROR(get_logger(),
                "Backup odom not available, cannot switch!");
            return;
        }

        tf2::Transform Twa = odomToTf(curr_a_);
        tf2::Transform Twb = odomToTf(curr_b_);

        T_world_b_offset_ = Twa * Twb.inverse();
        use_backup_ = true;

        RCLCPP_WARN(get_logger(),
            "Switched to BACKUP odometry (one-way)");
    }

    void publish()
    {
        Odometry out;

        if (!use_backup_) {
            out = curr_a_;
        } else {
            tf2::Transform Tb = odomToTf(curr_b_);
            tf2::Transform Tw = T_world_b_offset_ * Tb;

            out = curr_b_;
            out.pose.pose.position.x = Tw.getOrigin().x();
            out.pose.pose.position.y = Tw.getOrigin().y();
            out.pose.pose.position.z = Tw.getOrigin().z();
            out.pose.pose.orientation = tf2::toMsg(Tw.getRotation());
        }

        pub_->publish(out);
    }

    /* ======================= 工具函数 ======================= */

    tf2::Transform odomToTf(const Odometry& o)
    {
        tf2::Transform T;
        tf2::Quaternion q;
        tf2::fromMsg(o.pose.pose.orientation, q);
        T.setRotation(q);
        T.setOrigin(tf2::Vector3(
            o.pose.pose.position.x,
            o.pose.pose.position.y,
            o.pose.pose.position.z));
        return T;
    }

    double getYaw(const geometry_msgs::msg::Quaternion& q)
    {
        tf2::Quaternion tq;
        tf2::fromMsg(q, tq);
        double r, p, y;
        tf2::Matrix3x3(tq).getRPY(r, p, y);
        return y;
    }

    double normalize(double a)
    {
        while (a > M_PI) a -= 2 * M_PI;
        while (a < -M_PI) a += 2 * M_PI;
        return a;
    }
};

/* ======================= main ======================= */

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPreprocessor>());
    rclcpp::shutdown();
    return 0;
}

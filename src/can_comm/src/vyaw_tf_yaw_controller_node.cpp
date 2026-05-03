#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <string>
#include <algorithm>

using namespace std::chrono_literals;

class VyawTfYawControllerNode final : public rclcpp::Node
{
public:
  VyawTfYawControllerNode()
  : rclcpp::Node("vyaw_tf_yaw_controller"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_, this)
  {
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("base_frame", "base_link");

    // 方向相关参数全部使用角度制
    this->declare_parameter<double>("yaw_tolerance_deg", 3.0);

    // Pose of NLJG in map frame.
    this->declare_parameter<double>("NLJG_pose_x", 0.0);
    this->declare_parameter<double>("NLJG_pose_y", 0.0);
    this->declare_parameter<double>("NLJG_pose_yaw_deg", 0.0);

    // Pose of outpost in map frame.
    this->declare_parameter<double>("outpost_pose_x", 0.0);
    this->declare_parameter<double>("outpost_pose_y", 0.0);
    this->declare_parameter<double>("outpost_pose_yaw_deg", 0.0);

    // 旋转速度约束（绝对值）
    this->declare_parameter<double>("min_abs_vyaw", 0.15);
    this->declare_parameter<double>("max_abs_vyaw", 1.0);

    // 简单比例控制增益（基于弧度误差）
    this->declare_parameter<double>("k_p", 1.2);

    this->declare_parameter<double>("control_frequency", 30.0);

    cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();

    yaw_tolerance_deg_ = std::abs(this->get_parameter("yaw_tolerance_deg").as_double());
    
    NLJG_pose_x_ = this->get_parameter("NLJG_pose_x").as_double();
    NLJG_pose_y_ = this->get_parameter("NLJG_pose_y").as_double();
    NLJG_pose_yaw_deg_ = this->get_parameter("NLJG_pose_yaw_deg").as_double();

    outpost_pose_x_ = this->get_parameter("outpost_pose_x").as_double();
    outpost_pose_y_ = this->get_parameter("outpost_pose_y").as_double();
    outpost_pose_yaw_deg_ = this->get_parameter("outpost_pose_yaw_deg").as_double();

    min_abs_vyaw_ = std::abs(this->get_parameter("min_abs_vyaw").as_double());
    max_abs_vyaw_ = std::abs(this->get_parameter("max_abs_vyaw").as_double());
    k_p_ = this->get_parameter("k_p").as_double();

    control_frequency_ = this->get_parameter("control_frequency").as_double();

    if (max_abs_vyaw_ < min_abs_vyaw_) {
      RCLCPP_WARN(this->get_logger(), "max_abs_vyaw < min_abs_vyaw, swapping them automatically");
      std::swap(min_abs_vyaw_, max_abs_vyaw_);
    }

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    yaw_enable_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/yaw_controller",
      10,
      std::bind(&VyawTfYawControllerNode::onYawEnable, this, std::placeholders::_1));

    const double freq = std::max(control_frequency_, 1.0);
    const auto period = std::chrono::duration<double>(1.0 / freq);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&VyawTfYawControllerNode::onTimer, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Started. target_yaw will be computed from pose target, tolerance=%.2f deg, vyaw[min,max]=[%.3f, %.3f]",
      yaw_tolerance_deg_,
      min_abs_vyaw_,
      max_abs_vyaw_);

    RCLCPP_INFO(
      this->get_logger(),
      "NLJG_pose: x=%.3f, y=%.3f, yaw_deg=%.2f",
      NLJG_pose_x_, NLJG_pose_y_, NLJG_pose_yaw_deg_);

    RCLCPP_INFO(
      this->get_logger(),
      "outpost_pose: x=%.3f, y=%.3f, yaw_deg=%.2f",
      outpost_pose_x_, outpost_pose_y_, outpost_pose_yaw_deg_);

    RCLCPP_INFO(
      this->get_logger(),
      "Waiting for /yaw_controller msg (0=NLJG, 1=outpost) to start one-shot yaw control.");
  }

  ~VyawTfYawControllerNode() override
  {
    // Only stop the robot if we were actively controlling when destructed.
    if (active_) {
      publishVyaw(0.0);
    }
  }

private:
  static double normalizeAngleRad(const double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  static double rad2deg(const double rad)
  {
    return rad * 180.0 / M_PI;
  }

  static double deg2rad(const double deg)
  {
    return deg * M_PI / 180.0;
  }

  void publishVyaw(const double value)
  {
    geometry_msgs::msg::Twist msg;
    msg.angular.z = value;
    cmd_vel_pub_->publish(msg);
  }

  bool lookupCurrentPoseMap(double & x_out, double & y_out, double & yaw_out)
  {
    try {
      const auto tf = tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
      x_out = tf.transform.translation.x;
      y_out = tf.transform.translation.y;
      tf2::Quaternion q;
      q.setX(tf.transform.rotation.x);
      q.setY(tf.transform.rotation.y);
      q.setZ(tf.transform.rotation.z);
      q.setW(tf.transform.rotation.w);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      yaw_out = yaw;
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "TF lookup failed (%s->%s): %s",
        map_frame_.c_str(),
        base_frame_.c_str(),
        ex.what());
      return false;
    }
  }

  bool computeTargetYawRad(
    const double base_x_map,
    const double base_y_map,
    double & target_yaw_rad_out)
  {
    // Compute target yaw from right-triangle geometry:
    // A=(target_x_,target_y_), B=(base_x,base_y), C=(target_x_,base_y)
    // AC = |base_y - target_y_|, BC = |target_x_ - base_x|
    // theta = atan(AC/BC) (angle at B, keep positive)
    // target_yaw = 180deg - theta
    const double bc = std::abs(target_x_ - base_x_map);
    const double ac = std::abs(base_y_map - target_y_);
    if (bc < 1e-9) {
      // Degenerate (A and B share x); theta -> 90deg.
      target_yaw_rad_out = deg2rad(90.0);
      return true;
    }

    const double theta_rad = std::atan2(ac, bc);  // positive
    const double target_deg = 180.0 - rad2deg(theta_rad);
    target_yaw_rad_out = normalizeAngleRad(deg2rad(target_deg));
    return true;
  }

  void onYawEnable(const std_msgs::msg::Int32::SharedPtr msg)
  {
    if (msg->data == 0) {
      target_x_ = NLJG_pose_x_;
      target_y_ = NLJG_pose_y_;
      active_ = true;
      reached_once_ = false;
      has_target_yaw_ = false;
      RCLCPP_INFO(this->get_logger(), "Triggered by /yaw_controller=0. Using NLJG_pose.");
    } else if (msg->data == 1) {
      target_x_ = outpost_pose_x_;
      target_y_ = outpost_pose_y_;
      active_ = true;
      reached_once_ = false;
      has_target_yaw_ = false;
      RCLCPP_INFO(this->get_logger(), "Triggered by /yaw_controller=1. Using outpost_pose.");
    }
  }

  void onTimer()
  {
    // Only publish vyaw while executing; otherwise stay silent.
    if (!active_ || reached_once_) {
      return;
    }

    double base_x_map = 0.0;
    double base_y_map = 0.0;
    double current_yaw_rad = 0.0;
    if (!lookupCurrentPoseMap(base_x_map, base_y_map, current_yaw_rad)) {
      publishVyaw(0.0);
      return;
    }

    if (!has_target_yaw_) {
      if (!computeTargetYawRad(base_x_map, base_y_map, target_yaw_rad_)) {
        publishVyaw(0.0);
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          1000,
          "Target yaw computation failed (base too close to a?)");
        return;
      }
      has_target_yaw_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "Computed target yaw: %.2f deg",
        rad2deg(target_yaw_rad_));
    }

    const double target_yaw_rad = target_yaw_rad_;

    const double err_rad = normalizeAngleRad(target_yaw_rad - current_yaw_rad);
    const double err_deg = rad2deg(err_rad);

    if (std::abs(err_deg) <= yaw_tolerance_deg_) {
      // Publish zero once to stop the robot, then latch success.
      publishVyaw(0.0);
      reached_once_ = true;
      active_ = false;

      RCLCPP_INFO(
        this->get_logger(),
        "Reached target yaw (latched). current=%.2f deg, target=%.2f deg, err=%.2f deg",
        rad2deg(current_yaw_rad),
        rad2deg(target_yaw_rad),
        err_deg);

      return;
    }

    const double raw_cmd = k_p_ * std::abs(err_rad);
    const double cmd_abs = std::min(max_abs_vyaw_, std::max(min_abs_vyaw_, raw_cmd));
    const double cmd = (err_rad > 0.0) ? cmd_abs : -cmd_abs;
    publishVyaw(cmd);
  }

private:
  std::string cmd_vel_topic_;
  std::string map_frame_;
  std::string base_frame_;
  double yaw_tolerance_deg_ = 3.0;

  double target_x_ = 0.0;
  double target_y_ = 0.0;

  double NLJG_pose_x_ = 0.0;
  double NLJG_pose_y_ = 0.0;
  double NLJG_pose_yaw_deg_ = 0.0;

  double outpost_pose_x_ = 0.0;
  double outpost_pose_y_ = 0.0;
  double outpost_pose_yaw_deg_ = 0.0;

  double min_abs_vyaw_ = 0.15;
  double max_abs_vyaw_ = 1.0;
  double k_p_ = 1.2;

  double control_frequency_ = 30.0;

  bool active_ = false;
  bool reached_once_ = false;

  bool has_target_yaw_ = false;
  double target_yaw_rad_ = 0.0;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr yaw_enable_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VyawTfYawControllerNode>());
  rclcpp::shutdown();
  return 0;
}

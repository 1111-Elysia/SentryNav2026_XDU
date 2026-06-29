#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <string>

using namespace std::chrono_literals;

class VyawTfYawControllerNode final : public rclcpp::Node
{
public:
  VyawTfYawControllerNode()
  : rclcpp::Node("vyaw_tf_yaw_controller"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_, this)
  {
    this->declare_parameter<std::string>("target_yaw_topic", "/target_yaw");
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("base_frame", "base_link");

    // Pose of NLJG in map frame.
    this->declare_parameter<double>("NLJG_pose_x", 0.0);
    this->declare_parameter<double>("NLJG_pose_y", 0.0);

    // Pose of outpost in map frame.
    this->declare_parameter<double>("outpost_pose_x", 0.0);
    this->declare_parameter<double>("outpost_pose_y", 0.0);

    this->declare_parameter<double>("publish_rate_hz", 30.0);

    target_yaw_topic_ = this->get_parameter("target_yaw_topic").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();

    NLJG_pose_x_ = this->get_parameter("NLJG_pose_x").as_double();
    NLJG_pose_y_ = this->get_parameter("NLJG_pose_y").as_double();

    outpost_pose_x_ = this->get_parameter("outpost_pose_x").as_double();
    outpost_pose_y_ = this->get_parameter("outpost_pose_y").as_double();

    double publish_rate_hz = this->get_parameter("publish_rate_hz").as_double();
    if (publish_rate_hz <= 1e-6) publish_rate_hz = 30.0;

    target_yaw_pub_ = this->create_publisher<std_msgs::msg::Float32>(target_yaw_topic_, 10);

    yaw_enable_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/yaw_controller",
      10,
      std::bind(&VyawTfYawControllerNode::onYawEnable, this, std::placeholders::_1));

    const double freq = std::max(publish_rate_hz, 1.0);
    const auto period = std::chrono::duration<double>(1.0 / freq);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&VyawTfYawControllerNode::onTimer, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Started. target_yaw on %s @ %.1f Hz",
      target_yaw_topic_.c_str(), freq);

    RCLCPP_INFO(
      this->get_logger(),
      "NLJG_pose: x=%.3f, y=%.3f",
      NLJG_pose_x_, NLJG_pose_y_);

    RCLCPP_INFO(
      this->get_logger(),
      "outpost_pose: x=%.3f, y=%.3f",
      outpost_pose_x_, outpost_pose_y_);

    RCLCPP_INFO(
      this->get_logger(),
      "Waiting for /yaw_controller msg (0=NLJG, 1=outpost) to start.");
  }

  ~VyawTfYawControllerNode() override
  {
    if (active_) {
      publishTargetYaw(0.0);
    }
  }

private:
  static double rad2deg(const double rad)
  {
    return rad * 180.0 / M_PI;
  }

  void publishTargetYaw(const double target_yaw_deg)
  {
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(target_yaw_deg);
    target_yaw_pub_->publish(msg);
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

  double computeTargetYawDeg(const double base_x_map, const double base_y_map)
  {
    // atan2 直接给出地图坐标系下的绝对朝向角，区间 [-π, π]
    const double target_yaw_rad = std::atan2(target_y_ - base_y_map, target_x_ - base_x_map);
    return rad2deg(target_yaw_rad);
  }

  void onYawEnable(const std_msgs::msg::Int32::SharedPtr msg)
  {
    if (msg->data == 0) {
      target_x_ = NLJG_pose_x_;
      target_y_ = NLJG_pose_y_;
      active_ = true;
      has_target_yaw_ = false;
      RCLCPP_INFO(this->get_logger(), "Triggered by /yaw_controller=0. Using NLJG_pose.");
    } else if (msg->data == 1) {
      target_x_ = outpost_pose_x_;
      target_y_ = outpost_pose_y_;
      active_ = true;
      has_target_yaw_ = false;
      RCLCPP_INFO(this->get_logger(), "Triggered by /yaw_controller=1. Using outpost_pose.");
    }
  }

  void onTimer()
  {
    if (!active_) {
      return;
    }

    double base_x_map = 0.0;
    double base_y_map = 0.0;
    double current_yaw_rad = 0.0;
    if (!lookupCurrentPoseMap(base_x_map, base_y_map, current_yaw_rad)) {
      return;
    }

    // 首次：用当前机器人位置计算目标角度，锁定不再变化
    if (!has_target_yaw_) {
      target_yaw_deg_ = computeTargetYawDeg(base_x_map, base_y_map);
      has_target_yaw_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "Computed target yaw: %.2f deg (locked, keeps publishing)",
        target_yaw_deg_);
    }

    // 持续发送锁定的目标角度，直到下次 /yaw_controller 触发
    publishTargetYaw(target_yaw_deg_);
  }

private:
  std::string target_yaw_topic_;
  std::string map_frame_;
  std::string base_frame_;

  double target_x_ = 0.0;
  double target_y_ = 0.0;

  double NLJG_pose_x_ = 0.0;
  double NLJG_pose_y_ = 0.0;

  double outpost_pose_x_ = 0.0;
  double outpost_pose_y_ = 0.0;

  bool active_ = false;
  bool has_target_yaw_ = false;
  double target_yaw_deg_ = 0.0;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_yaw_pub_;
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

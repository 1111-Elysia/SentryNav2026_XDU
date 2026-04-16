#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace {

double clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(v, hi));
}

tf2::Vector3 lpf(const tf2::Vector3 & prev, const tf2::Vector3 & cur, double alpha)
{
  return prev * (1.0 - alpha) + cur * alpha;
}

}  // namespace

class SlopeProcessNode : public rclcpp::Node
{
public:
  SlopeProcessNode()
  : Node("slope_process"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    last_cmd_time_(0, 0, this->get_clock()->get_clock_type())
  {
    declare_parameter<std::string>("input_cmd_topic", "/cmd_vel");
    declare_parameter<std::string>("output_cmd_topic", "/cmd_vel_good");
    declare_parameter<std::string>("imu_topic", "/livox/imu");
    declare_parameter<double>("uphill_speed", 0.2);

    declare_parameter<double>("slope_threshold_deg", 8.0);
    declare_parameter<int>("init_sample_count", 200);

    declare_parameter<double>("publish_rate", 50.0);
    declare_parameter<double>("cmd_timeout_sec", 0.2);

    declare_parameter<bool>("use_tf", true);
    declare_parameter<std::string>("base_frame", "base_link");

    declare_parameter<double>("acc_lpf_alpha", 0.1);
    declare_parameter<double>("min_xy_acc_norm", 0.15);

    input_cmd_topic_ = get_parameter("input_cmd_topic").as_string();
    output_cmd_topic_ = get_parameter("output_cmd_topic").as_string();
    imu_topic_ = get_parameter("imu_topic").as_string();

    uphill_speed_ = get_parameter("uphill_speed").as_double();
    slope_threshold_deg_ = get_parameter("slope_threshold_deg").as_double();
    const int64_t init_samples = get_parameter("init_sample_count").as_int();
    init_sample_count_ = static_cast<int>(std::max<int64_t>(1, init_samples));

    publish_rate_ = std::max(1e-3, get_parameter("publish_rate").as_double());
    cmd_timeout_sec_ = std::max(0.0, get_parameter("cmd_timeout_sec").as_double());

    use_tf_ = get_parameter("use_tf").as_bool();
    base_frame_ = get_parameter("base_frame").as_string();

    acc_lpf_alpha_ = clamp(get_parameter("acc_lpf_alpha").as_double(), 0.0, 1.0);
    min_xy_acc_norm_ = std::max(0.0, get_parameter("min_xy_acc_norm").as_double());

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      input_cmd_topic_, 10,
      std::bind(&SlopeProcessNode::onCmdVel, this, std::placeholders::_1));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SlopeProcessNode::onImu, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_cmd_topic_, 10);

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SlopeProcessNode::onTimer, this));

    RCLCPP_INFO(get_logger(),
      "slope_process started | cmd_in=%s cmd_out=%s imu=%s uphill_speed=%.3f",
      input_cmd_topic_.c_str(), output_cmd_topic_.c_str(), imu_topic_.c_str(), uphill_speed_);
  }

private:
  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_ = *msg;
    last_cmd_time_ = now();
    have_cmd_ = true;
  }

  bool toBaseFrame(const sensor_msgs::msg::Imu & imu_msg, tf2::Vector3 & acc_out)
  {
    const auto & a = imu_msg.linear_acceleration;
    geometry_msgs::msg::Vector3Stamped a_in;
    a_in.header = imu_msg.header;
    a_in.vector.x = a.x;
    a_in.vector.y = a.y;
    a_in.vector.z = a.z;

    if (!use_tf_) {
      acc_out = tf2::Vector3(a.x, a.y, a.z);
      return true;
    }

    const std::string src_frame = imu_msg.header.frame_id;
    if (src_frame.empty() || src_frame == base_frame_) {
      acc_out = tf2::Vector3(a.x, a.y, a.z);
      return true;
    }

    try {
      // Use the latest transform if timestamps are not valid.
      const rclcpp::Time stamp = imu_msg.header.stamp.sec == 0 && imu_msg.header.stamp.nanosec == 0
        ? rclcpp::Time(0, 0, get_clock()->get_clock_type())
        : rclcpp::Time(imu_msg.header.stamp, get_clock()->get_clock_type());

      auto tf = tf_buffer_.lookupTransform(base_frame_, src_frame, stamp, rclcpp::Duration::from_seconds(0.05));
      geometry_msgs::msg::Vector3Stamped a_out;
      tf2::doTransform(a_in, a_out, tf);
      acc_out = tf2::Vector3(a_out.vector.x, a_out.vector.y, a_out.vector.z);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF failed (%s -> %s): %s", src_frame.c_str(), base_frame_.c_str(), ex.what());
      return false;
    }
  }

  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    tf2::Vector3 acc;
    if (!toBaseFrame(*msg, acc)) {
      return;
    }

    if (!have_filtered_acc_) {
      filtered_acc_ = acc;
      have_filtered_acc_ = true;
    } else {
      filtered_acc_ = lpf(filtered_acc_, acc, acc_lpf_alpha_);
    }

    const double norm = filtered_acc_.length();
    if (norm < 1e-3) {
      return;
    }

    tf2::Vector3 g_dir = filtered_acc_ / norm;

    if (!gravity_inited_) {
      init_sum_acc_ += g_dir;
      init_count_++;
      if (init_count_ >= init_sample_count_) {
        gravity_init_dir_ = init_sum_acc_ / static_cast<double>(init_count_);
        const double n0 = gravity_init_dir_.length();
        if (n0 < 1e-3) {
          init_sum_acc_.setZero();
          init_count_ = 0;
          RCLCPP_WARN(get_logger(), "gravity init failed (norm too small), retrying...");
          return;
        }
        gravity_init_dir_ /= n0;
        gravity_inited_ = true;
        RCLCPP_INFO(get_logger(),
          "gravity direction initialized with %d samples (frame=%s)",
          init_count_, base_frame_.c_str());
      }
      return;
    }

    // Tilt magnitude relative to initial gravity direction.
    const double dot = clamp(gravity_init_dir_.dot(g_dir), -1.0, 1.0);
    const double tilt_deg = std::acos(dot) * 180.0 / M_PI;

    // Uphill direction in base frame: use xy projection of current gravity direction.
    const double xy_norm = std::hypot(g_dir.x(), g_dir.y());
    bool slope_now = (tilt_deg >= slope_threshold_deg_) && (xy_norm >= min_xy_acc_norm_);

    if (slope_now) {
      uphill_dir_x_ = g_dir.x() / xy_norm;
      uphill_dir_y_ = g_dir.y() / xy_norm;
    }

    if (slope_now != slope_detected_) {
      slope_detected_ = slope_now;
      RCLCPP_INFO(get_logger(),
        "slope_detected=%s tilt=%.2fdeg uphill_dir=[%.2f, %.2f]", 
        slope_detected_ ? "true" : "false", tilt_deg, uphill_dir_x_, uphill_dir_y_);
    }
  }

  void onTimer()
  {
    geometry_msgs::msg::Twist cmd_out;

    const auto t = now();
    const bool cmd_fresh = have_cmd_ && ((t - last_cmd_time_).seconds() <= cmd_timeout_sec_);
    if (cmd_fresh) {
      cmd_out = last_cmd_;
    }

    if (gravity_inited_ && slope_detected_) {
      cmd_out.linear.x += uphill_speed_ * uphill_dir_x_;
      cmd_out.linear.y += uphill_speed_ * uphill_dir_y_;
    }

    cmd_pub_->publish(cmd_out);
  }

private:
  // Params
  std::string input_cmd_topic_;
  std::string output_cmd_topic_;
  std::string imu_topic_;
  std::string base_frame_;

  double uphill_speed_ = 0.2;
  double slope_threshold_deg_ = 8.0;
  int init_sample_count_ = 200;

  double publish_rate_ = 50.0;
  double cmd_timeout_sec_ = 0.2;

  bool use_tf_ = true;
  double acc_lpf_alpha_ = 0.1;
  double min_xy_acc_norm_ = 0.15;

  // IO
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // State
  geometry_msgs::msg::Twist last_cmd_;
  bool have_cmd_ = false;
  rclcpp::Time last_cmd_time_;

  bool have_filtered_acc_ = false;
  tf2::Vector3 filtered_acc_;

  bool gravity_inited_ = false;
  tf2::Vector3 gravity_init_dir_{0.0, 0.0, 1.0};
  tf2::Vector3 init_sum_acc_{0.0, 0.0, 0.0};
  int init_count_ = 0;

  bool slope_detected_ = false;
  double uphill_dir_x_ = 0.0;
  double uphill_dir_y_ = 0.0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlopeProcessNode>());
  rclcpp::shutdown();
  return 0;
}

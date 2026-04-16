#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sentry_msgs/msg/scan_mode.hpp"
#include "sentry_msgs/msg/vw.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
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

    // 从 TF 读取 map -> base_frame 来计算坡度（完全不依赖 IMU）
    declare_parameter<std::string>("map_frame", "map");

    // 当检测到坡度时，同时发布 vw（sentry_msgs/msg/Vw）
    declare_parameter<std::string>("vw_topic", "/vw_slope");
    declare_parameter<double>("uphill_vw", 0.0);

    // 坡度触发时，向 scan_mod_type 发送一次 false；退出坡度后再发送一次 true
    // 话题类型：sentry_msgs/msg/ScanMode（字段名 scan_mod_type）
    declare_parameter<std::string>("scan_mode_topic", "/scan_mod_type");

    declare_parameter<double>("uphill_speed", 0.2);

    // 进入/退出坡度阈值（带回差，避免在阈值附近抖动）
    declare_parameter<double>("slope_enter_threshold_deg", 8.0);
    declare_parameter<double>("slope_exit_threshold_deg", 6.0);
    // 兼容旧参数名（不建议继续使用）
    declare_parameter<double>("slope_threshold_deg", 8.0);

    // 坡度进入/退出确认计数（连续满足条件才切换状态）
    declare_parameter<int>("slope_on_confirm", 5);
    declare_parameter<int>("slope_off_confirm", 15);

    declare_parameter<int>("init_sample_count", 200);

    declare_parameter<double>("publish_rate", 50.0);
    declare_parameter<double>("cmd_timeout_sec", 0.2);

    declare_parameter<bool>("use_tf", true);
    declare_parameter<std::string>("base_frame", "base_link");

    // TF 查询超时（秒）
    declare_parameter<double>("tf_timeout_sec", 0.05);

    // TF 上方向（map 的 +Z 在 base_frame 下的方向）低通
    declare_parameter<double>("up_lpf_alpha", 0.2);

    // XY 投影最小阈值：接近平地时不更新上坡方向（避免噪声抖动）
    declare_parameter<double>("min_xy_up_norm", 0.15);
    // 兼容旧参数名（不建议继续使用）
    declare_parameter<double>("min_xy_acc_norm", 0.15);

    // 上坡方向低通，避免方向在两个象限间来回跳
    declare_parameter<double>("uphill_dir_lpf_alpha", 0.2);

    input_cmd_topic_ = get_parameter("input_cmd_topic").as_string();
    output_cmd_topic_ = get_parameter("output_cmd_topic").as_string();
    map_frame_ = get_parameter("map_frame").as_string();
    vw_topic_ = get_parameter("vw_topic").as_string();
    scan_mode_topic_ = get_parameter("scan_mode_topic").as_string();

    uphill_vw_ = get_parameter("uphill_vw").as_double();
    uphill_speed_ = get_parameter("uphill_speed").as_double();

    slope_enter_threshold_deg_ = get_parameter("slope_enter_threshold_deg").as_double();
    slope_exit_threshold_deg_ = get_parameter("slope_exit_threshold_deg").as_double();

    // 兼容旧参数 slope_threshold_deg：如果新参数没被改过但旧参数被改过，则用旧参数推导
    {
      constexpr double kDefaultEnter = 8.0;
      constexpr double kDefaultExit = 6.0;
      constexpr double kDefaultLegacy = 8.0;
      const double legacy = get_parameter("slope_threshold_deg").as_double();
      const bool new_is_default = (std::abs(slope_enter_threshold_deg_ - kDefaultEnter) < 1e-9) &&
        (std::abs(slope_exit_threshold_deg_ - kDefaultExit) < 1e-9);
      const bool legacy_changed = (std::abs(legacy - kDefaultLegacy) >= 1e-9);
      if (new_is_default && legacy_changed) {
        slope_enter_threshold_deg_ = legacy;
        slope_exit_threshold_deg_ = legacy - 2.0;
      }
    }

    // 确保回差方向正确
    if (slope_exit_threshold_deg_ > slope_enter_threshold_deg_) {
      std::swap(slope_exit_threshold_deg_, slope_enter_threshold_deg_);
    }

    const int64_t on_c = get_parameter("slope_on_confirm").as_int();
    const int64_t off_c = get_parameter("slope_off_confirm").as_int();
    slope_on_confirm_ = static_cast<int>(std::max<int64_t>(1, on_c));
    slope_off_confirm_ = static_cast<int>(std::max<int64_t>(1, off_c));

    const int64_t init_samples = get_parameter("init_sample_count").as_int();
    init_sample_count_ = static_cast<int>(std::max<int64_t>(1, init_samples));

    publish_rate_ = std::max(1e-3, get_parameter("publish_rate").as_double());
    cmd_timeout_sec_ = std::max(0.0, get_parameter("cmd_timeout_sec").as_double());

    use_tf_ = get_parameter("use_tf").as_bool();
    base_frame_ = get_parameter("base_frame").as_string();
    tf_timeout_sec_ = std::max(0.0, get_parameter("tf_timeout_sec").as_double());

    up_lpf_alpha_ = clamp(get_parameter("up_lpf_alpha").as_double(), 0.0, 1.0);
    min_xy_up_norm_ = std::max(0.0, get_parameter("min_xy_up_norm").as_double());
    // 兼容旧参数 min_xy_acc_norm
    {
      constexpr double kDefaultNew = 0.15;
      constexpr double kDefaultLegacy = 0.15;
      const double legacy = get_parameter("min_xy_acc_norm").as_double();
      const bool new_is_default = (std::abs(min_xy_up_norm_ - kDefaultNew) < 1e-9);
      const bool legacy_changed = (std::abs(legacy - kDefaultLegacy) >= 1e-9);
      if (new_is_default && legacy_changed) {
        min_xy_up_norm_ = legacy;
      }
    }

    uphill_dir_lpf_alpha_ = clamp(get_parameter("uphill_dir_lpf_alpha").as_double(), 0.0, 1.0);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      input_cmd_topic_, 10,
      std::bind(&SlopeProcessNode::onCmdVel, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_cmd_topic_, 10);

    if (!vw_topic_.empty()) {
      vw_pub_ = create_publisher<sentry_msgs::msg::Vw>(vw_topic_, 10);
    }

    if (!scan_mode_topic_.empty()) {
      scan_mode_pub_ = create_publisher<sentry_msgs::msg::ScanMode>(scan_mode_topic_, 10);
    }

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SlopeProcessNode::onTimer, this));

    RCLCPP_INFO(get_logger(),
      "slope_process started | cmd_in=%s cmd_out=%s map=%s base=%s uphill_speed=%.3f",
      input_cmd_topic_.c_str(), output_cmd_topic_.c_str(), map_frame_.c_str(), base_frame_.c_str(), uphill_speed_);
  }

private:
  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_ = *msg;
    last_cmd_time_ = now();
    have_cmd_ = true;
  }

  bool getUpDirFromTf(tf2::Vector3 & up_dir_out)
  {
    if (!use_tf_) {
      return false;
    }

    try {
      const rclcpp::Time stamp(0, 0, get_clock()->get_clock_type());
      auto tf = tf_buffer_.lookupTransform(
        map_frame_, base_frame_, stamp, rclcpp::Duration::from_seconds(tf_timeout_sec_));

      const auto & r = tf.transform.rotation;
      tf2::Quaternion q(r.x, r.y, r.z, r.w);
      if (q.length2() < 1e-12) {
        return false;
      }
      q.normalize();

      const tf2::Vector3 up_map(0.0, 0.0, 1.0);
      tf2::Vector3 up_base = tf2::quatRotate(q.inverse(), up_map);

      const double n = up_base.length();
      if (n < 1e-6) {
        return false;
      }

      up_dir_out = up_base / n;
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF failed (%s -> %s): %s", base_frame_.c_str(), map_frame_.c_str(), ex.what());
      return false;
    }
  }

  void updateSlopeStateFromTf()
  {
    tf2::Vector3 up_dir;
    if (!getUpDirFromTf(up_dir)) {
      return;
    }

    if (!have_filtered_up_) {
      filtered_up_dir_ = up_dir;
      have_filtered_up_ = true;
    } else {
      filtered_up_dir_ = lpf(filtered_up_dir_, up_dir, up_lpf_alpha_);
      const double n = filtered_up_dir_.length();
      if (n > 1e-6) {
        filtered_up_dir_ /= n;
      }
    }

    const tf2::Vector3 g_dir = filtered_up_dir_;

    if (!gravity_inited_) {
      init_sum_acc_ += g_dir;
      init_count_++;
      if (init_count_ >= init_sample_count_) {
        gravity_init_dir_ = init_sum_acc_ / static_cast<double>(init_count_);
        const double n0 = gravity_init_dir_.length();
        if (n0 < 1e-3) {
          init_sum_acc_.setZero();
          init_count_ = 0;
          RCLCPP_WARN(get_logger(), "up direction init failed (norm too small), retrying...");
          return;
        }
        gravity_init_dir_ /= n0;
        gravity_inited_ = true;
        RCLCPP_INFO(get_logger(),
          "up direction initialized with %d samples (map=%s base=%s)",
          init_count_, map_frame_.c_str(), base_frame_.c_str());
      }
      return;
    }

    // Tilt magnitude relative to initial up direction.
    const double dot = clamp(gravity_init_dir_.dot(g_dir), -1.0, 1.0);
    const double tilt_deg = std::acos(dot) * 180.0 / M_PI;

    // Uphill direction in base frame: use xy projection of current up direction.
    const double xy_norm = std::hypot(g_dir.x(), g_dir.y());

    const bool raw_slope = (tilt_deg >= slope_enter_threshold_deg_) && (xy_norm >= min_xy_up_norm_);
    const bool raw_flat = (tilt_deg <= slope_exit_threshold_deg_) || (xy_norm < min_xy_up_norm_);

    // 方向只在“认为在坡上/可能在坡上”时更新，避免平地噪声把方向翻来覆去
    if (raw_slope && xy_norm > 1e-6) {
      const double ux = g_dir.x() / xy_norm;
      const double uy = g_dir.y() / xy_norm;

      if (!have_uphill_dir_) {
        uphill_dir_x_ = ux;
        uphill_dir_y_ = uy;
        have_uphill_dir_ = true;
      } else {
        uphill_dir_x_ = uphill_dir_x_ * (1.0 - uphill_dir_lpf_alpha_) + ux * uphill_dir_lpf_alpha_;
        uphill_dir_y_ = uphill_dir_y_ * (1.0 - uphill_dir_lpf_alpha_) + uy * uphill_dir_lpf_alpha_;
        const double n = std::hypot(uphill_dir_x_, uphill_dir_y_);
        if (n > 1e-6) {
          uphill_dir_x_ /= n;
          uphill_dir_y_ /= n;
        }
      }
    }

    // 状态去抖：连续满足条件才切换，避免退出坡后在阈值附近来回触发
    bool slope_next = slope_detected_;
    if (!slope_detected_) {
      if (raw_slope) {
        slope_on_count_++;
      } else {
        slope_on_count_ = 0;
      }
      if (slope_on_count_ >= slope_on_confirm_) {
        slope_next = true;
        slope_on_count_ = 0;
        slope_off_count_ = 0;
      }
    } else {
      if (raw_flat) {
        slope_off_count_++;
      } else {
        slope_off_count_ = 0;
      }
      if (slope_off_count_ >= slope_off_confirm_) {
        slope_next = false;
        slope_on_count_ = 0;
        slope_off_count_ = 0;
      }
    }

    if (slope_next != slope_detected_) {
      if (scan_mode_pub_) {
        sentry_msgs::msg::ScanMode m;
        m.scan_mod_type = !slope_next;
        scan_mode_pub_->publish(m);
      }

      slope_detected_ = slope_next;
      RCLCPP_INFO(get_logger(),
        "slope_detected=%s tilt=%.2fdeg uphill_dir=[%.2f, %.2f]", 
        slope_detected_ ? "true" : "false", tilt_deg, uphill_dir_x_, uphill_dir_y_);
    }
  }

  void onTimer()
  {
    // 用 TF(map->base_frame) 更新坡度状态
    updateSlopeStateFromTf();

    geometry_msgs::msg::Twist cmd_out;

    const auto t = now();
    const bool cmd_fresh = have_cmd_ && ((t - last_cmd_time_).seconds() <= cmd_timeout_sec_);
    if (cmd_fresh) {
      cmd_out = last_cmd_;
    }

    const bool uphill_active = gravity_inited_ && slope_detected_;
    if (uphill_active) {
      cmd_out.linear.x += uphill_speed_ * uphill_dir_x_;
      cmd_out.linear.y += uphill_speed_ * uphill_dir_y_;
    }

    cmd_pub_->publish(cmd_out);

    // 上坡辅助生效时，同步发布 vw；退出上坡时发布一次 0 清零
    if (vw_pub_) {
      if (uphill_active) {
        sentry_msgs::msg::Vw vw_msg;
        vw_msg.vw = static_cast<float>(uphill_vw_);
        vw_pub_->publish(vw_msg);
        vw_nonzero_published_ = (std::abs(uphill_vw_) > 1e-6);
      } else if (vw_nonzero_published_) {
        sentry_msgs::msg::Vw vw_msg;
        vw_msg.vw = 0.0f;
        vw_pub_->publish(vw_msg);
        vw_nonzero_published_ = false;
      }
    }
  }

private:
  // Params
  std::string input_cmd_topic_;
  std::string output_cmd_topic_;
  std::string map_frame_;
  std::string vw_topic_;
  std::string scan_mode_topic_;
  std::string base_frame_;

  double uphill_speed_ = 0.2;
  double uphill_vw_ = 0.0;

  double slope_enter_threshold_deg_ = 8.0;
  double slope_exit_threshold_deg_ = 6.0;
  int slope_on_confirm_ = 5;
  int slope_off_confirm_ = 15;

  int init_sample_count_ = 200;

  double publish_rate_ = 50.0;
  double cmd_timeout_sec_ = 0.2;

  bool use_tf_ = true;
  double tf_timeout_sec_ = 0.05;
  double up_lpf_alpha_ = 0.2;
  double min_xy_up_norm_ = 0.15;
  double uphill_dir_lpf_alpha_ = 0.2;

  // IO
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<sentry_msgs::msg::Vw>::SharedPtr vw_pub_;
  rclcpp::Publisher<sentry_msgs::msg::ScanMode>::SharedPtr scan_mode_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // State
  geometry_msgs::msg::Twist last_cmd_;
  bool have_cmd_ = false;
  rclcpp::Time last_cmd_time_;

  bool have_filtered_up_ = false;
  tf2::Vector3 filtered_up_dir_{0.0, 0.0, 1.0};

  bool gravity_inited_ = false;
  tf2::Vector3 gravity_init_dir_{0.0, 0.0, 1.0};
  tf2::Vector3 init_sum_acc_{0.0, 0.0, 0.0};
  int init_count_ = 0;

  bool slope_detected_ = false;
  int slope_on_count_ = 0;
  int slope_off_count_ = 0;

  bool have_uphill_dir_ = false;
  double uphill_dir_x_ = 0.0;
  double uphill_dir_y_ = 0.0;

  bool vw_nonzero_published_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlopeProcessNode>());
  rclcpp::shutdown();
  return 0;
}

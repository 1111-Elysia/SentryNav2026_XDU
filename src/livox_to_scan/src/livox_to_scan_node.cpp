#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>

#include <cmath>
#include <limits>

class LivoxToScanNode : public rclcpp::Node
{
public:
  LivoxToScanNode()
  : Node("livox_to_scan_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    /* ---------------- 参数 ---------------- */
    this->declare_parameter("min_height", -1.5);
    this->declare_parameter("max_height",  1.5);
    this->declare_parameter("angle_min",  -M_PI);
    this->declare_parameter("angle_max",   M_PI);
    this->declare_parameter("angle_increment", 0.005);
    this->declare_parameter("scan_time", 0.1);
    this->declare_parameter("range_min", 0.1);
    this->declare_parameter("range_max", 100.0);
    this->declare_parameter("target_frame", "base_link");

    min_height_       = this->get_parameter("min_height").as_double();
    max_height_       = this->get_parameter("max_height").as_double();
    angle_min_        = this->get_parameter("angle_min").as_double();
    angle_max_        = this->get_parameter("angle_max").as_double();
    angle_increment_  = this->get_parameter("angle_increment").as_double();
    scan_time_        = this->get_parameter("scan_time").as_double();
    range_min_        = this->get_parameter("range_min").as_double();
    range_max_        = this->get_parameter("range_max").as_double();
    target_frame_     = this->get_parameter("target_frame").as_string();

    beam_size_ = static_cast<size_t>(
        std::ceil((angle_max_ - angle_min_) / angle_increment_));

    /* ---------------- 通信 ---------------- */
    cloud_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
      "/livox/lidar",
      rclcpp::SensorDataQoS(),
      std::bind(&LivoxToScanNode::cloudCallback, this, std::placeholders::_1));

    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      "/scan",
      rclcpp::SensorDataQoS());

    RCLCPP_INFO(this->get_logger(),
      "Livox → LaserScan 节点已启动 | target_frame: %s",
      target_frame_.c_str());
  }

private:
  /* ---------------- 核心回调 ---------------- */
  void cloudCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped transform;
    bool tf_ok = true;

    /* ------ 关键点：使用 TimePointZero ---------- */
    try {
      transform = tf_buffer_.lookupTransform(
        target_frame_,
        msg->header.frame_id,
        tf2::TimePointZero
      );
    } catch (const tf2::TransformException &ex) {
      tf_ok = false;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "TF unavailable (%s -> %s): %s",
        msg->header.frame_id.c_str(),
        target_frame_.c_str(),
        ex.what());
    }

    sensor_msgs::msg::LaserScan scan;

    /* ---------- 时间戳修正与延迟保护 ---------- */
    rclcpp::Time start_stamp = msg->header.stamp;
    rclcpp::Time end_stamp = start_stamp + rclcpp::Duration::from_seconds(scan_time_);
    
    rclcpp::Time now = this->now();
    double lag = (now - end_stamp).seconds();
    
    // 如果滞后超过 0.15s (约1.5帧)，说明处理队列堵塞或 TF 已经跑到了未来
    // 强制将时间戳拉回到 now()，确保 Costmap 能接受数据进行避障
    if (lag > 0.15) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Scan data lag detected (%.3fs). Resetting stamp to now() to prevent drop.", lag);
        end_stamp = now;
    } else if (lag < -1.0) {
       // 防止严重的系统时间回跳或错误的时间戳
       end_stamp = now;
    }

    initScan(scan, end_stamp);

    /* ---------- TF 不可用：仍然发布空 scan ---------- */
    if (!tf_ok) {
      scan_pub_->publish(scan);
      return;
    }

    /* ---------- 点云 → scan ---------- */
    for (const auto &pt : msg->points)
    {
      geometry_msgs::msg::PointStamped p_in, p_out;
      p_in.header = msg->header;
      p_in.point.x = pt.x;
      p_in.point.y = pt.y;
      p_in.point.z = pt.z;

      tf2::doTransform(p_in, p_out, transform);

      const float x = p_out.point.x;
      const float y = p_out.point.y;
      const float z = p_out.point.z;

      if (z < min_height_ || z > max_height_)
        continue;

      const float range = std::hypot(x, y);
      if (range < range_min_ || range > range_max_)
        continue;

      const float angle = std::atan2(y, x);
      if (angle < angle_min_ || angle > angle_max_)
        continue;

      const int idx =
        static_cast<int>((angle - angle_min_) / angle_increment_);

      if (idx >= 0 && idx < static_cast<int>(beam_size_)) {
        if (range < scan.ranges[idx]) {
          scan.ranges[idx] = range;
          scan.intensities[idx] = static_cast<float>(pt.reflectivity);
        }
      }
    }

    scan_pub_->publish(scan);
  }

  /* ---------------- scan 初始化 ---------------- */
  void initScan(sensor_msgs::msg::LaserScan &scan, const rclcpp::Time &stamp)
  {
    scan.header.stamp = stamp;
    scan.header.frame_id = target_frame_;

    scan.angle_min = angle_min_;
    scan.angle_max = angle_max_;
    scan.angle_increment = angle_increment_;
    scan.scan_time = scan_time_;
    scan.range_min = range_min_;
    scan.range_max = range_max_;

    scan.ranges.assign(beam_size_,
      std::numeric_limits<float>::infinity());
    scan.intensities.assign(beam_size_, 0.0f);
  }

  /* ---------------- 成员变量 ---------------- */
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  double min_height_, max_height_;
  double angle_min_, angle_max_, angle_increment_;
  double scan_time_, range_min_, range_max_;
  std::string target_frame_;
  size_t beam_size_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxToScanNode>());
  rclcpp::shutdown();
  return 0;
}

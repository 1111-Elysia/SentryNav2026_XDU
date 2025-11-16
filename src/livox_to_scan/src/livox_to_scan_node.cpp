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
    // 参数
    this->declare_parameter("min_height", -0.5);
    this->declare_parameter("max_height", 2.0);
    this->declare_parameter("angle_min", -M_PI);
    this->declare_parameter("angle_max", M_PI);
    this->declare_parameter("angle_increment", 0.005);
    this->declare_parameter("scan_time", 0.1);
    this->declare_parameter("range_min", 0.1);
    this->declare_parameter("range_max", 100.0);
    this->declare_parameter("target_frame", "base_link"); // 固定参考系

    min_height_ = this->get_parameter("min_height").as_double();
    max_height_ = this->get_parameter("max_height").as_double();
    angle_min_ = this->get_parameter("angle_min").as_double();
    angle_max_ = this->get_parameter("angle_max").as_double();
    angle_increment_ = this->get_parameter("angle_increment").as_double();
    scan_time_ = this->get_parameter("scan_time").as_double();
    range_min_ = this->get_parameter("range_min").as_double();
    range_max_ = this->get_parameter("range_max").as_double();
    target_frame_ = this->get_parameter("target_frame").as_string();

    cloud_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        "/livox/lidar", 10,
        std::bind(&LivoxToScanNode::cloudCallback, this, std::placeholders::_1));

    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "Livox → LaserScan 启动 (target_frame: %s)", target_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "========================================");
  }

private:
  void cloudCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
  {
    // 获取从 livox_frame 到 target_frame 的 TF
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
          target_frame_, msg->header.frame_id, 
          msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "无法获取 TF %s → %s: %s", 
                           msg->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
      return;
    }

    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = msg->header.stamp; 
    scan.header.frame_id = target_frame_; // 输出在固定参考系

    scan.angle_min = angle_min_;
    scan.angle_max = angle_max_;
    scan.angle_increment = angle_increment_;
    scan.scan_time = scan_time_;
    scan.range_min = range_min_;
    scan.range_max = range_max_;

    uint32_t size = std::ceil((angle_max_ - angle_min_) / angle_increment_);
    scan.ranges.assign(size, std::numeric_limits<float>::infinity());
    scan.intensities.assign(size, 0.0);

    for (const auto &pt : msg->points)
    {
      // 原始点（livox_frame）
      geometry_msgs::msg::PointStamped pt_in, pt_out;
      pt_in.header = msg->header;
      pt_in.point.x = pt.x;
      pt_in.point.y = pt.y;
      pt_in.point.z = pt.z;

      // 转换到 target_frame
      tf2::doTransform(pt_in, pt_out, transform);

      float x = pt_out.point.x;
      float y = pt_out.point.y;
      float z = pt_out.point.z;

      // 高度过滤（在 target_frame 中）
      if (z < min_height_ || z > max_height_) continue;

      float range = std::sqrt(x*x + y*y);
      if (range < range_min_ || range > range_max_) continue;

      // 角度相对于 target_frame 的 +X
      float angle = std::atan2(y, x);
      if (angle < angle_min_ || angle > angle_max_) continue;

      int idx = static_cast<int>((angle - angle_min_) / angle_increment_);
      if (idx >= 0 && idx < static_cast<int>(size)) {
        if (range < scan.ranges[idx]) {
          scan.ranges[idx] = range;
          scan.intensities[idx] = pt.reflectivity;
        }
      }
    }

    scan_pub_->publish(scan);
  }

  // 成员变量
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  
  double min_height_, max_height_;
  double angle_min_, angle_max_;
  double angle_increment_, scan_time_;
  double range_min_, range_max_;
  std::string target_frame_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxToScanNode>());
  rclcpp::shutdown();
  return 0;
}
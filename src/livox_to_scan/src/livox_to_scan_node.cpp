#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <cmath>
#include <limits>

class LivoxToScanNode : public rclcpp::Node
{
public:
  LivoxToScanNode() : Node("livox_to_scan_node")
  {
    this->declare_parameter("min_height", -0.5);
    this->declare_parameter("max_height", 2.0);
    this->declare_parameter("angle_min", -M_PI);
    this->declare_parameter("angle_max", M_PI);
    this->declare_parameter("angle_increment", 0.005);
    this->declare_parameter("scan_time", 0.1);
    this->declare_parameter("range_min", 0.1);
    this->declare_parameter("range_max", 100.0);
    this->declare_parameter("input_topic", "/livox/lidar");
    this->declare_parameter("output_topic", "/scan");
    this->declare_parameter("queue_size", 50);
    this->declare_parameter("debug_points", 10);

    min_height_ = this->get_parameter("min_height").as_double();
    max_height_ = this->get_parameter("max_height").as_double();
    angle_min_ = this->get_parameter("angle_min").as_double();
    angle_max_ = this->get_parameter("angle_max").as_double();
    angle_increment_ = this->get_parameter("angle_increment").as_double();
    scan_time_ = this->get_parameter("scan_time").as_double();
    range_min_ = this->get_parameter("range_min").as_double();
    range_max_ = this->get_parameter("range_max").as_double();
    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    queue_size_ = this->get_parameter("queue_size").as_int();
    debug_points_ = this->get_parameter("debug_points").as_int();

    cloud_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        input_topic_, queue_size_,
        std::bind(&LivoxToScanNode::cloudCallback, this, std::placeholders::_1));

    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, queue_size_);

    RCLCPP_INFO(this->get_logger(), "============================================");
    RCLCPP_INFO(this->get_logger(), "Livox to Scan 节点已启动");
    RCLCPP_WARN(this->get_logger(), "  不转换坐标！直接使用雷达原始坐标系");
    RCLCPP_WARN(this->get_logger(), "  frame_id = livox_frame");
    RCLCPP_INFO(this->get_logger(), "============================================");
  }

private:
  void cloudCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr livox_msg)
  {
    auto scan_msg = sensor_msgs::msg::LaserScan();
    scan_msg.header.stamp = this->get_clock()->now();
    scan_msg.header.frame_id = livox_msg->header.frame_id;  // 使用雷达原始 frame_id
    scan_msg.angle_min = angle_min_;
    scan_msg.angle_max = angle_max_;
    scan_msg.angle_increment = angle_increment_;
    scan_msg.time_increment = 0.0;
    scan_msg.scan_time = scan_time_;
    scan_msg.range_min = range_min_;
    scan_msg.range_max = range_max_;

    uint32_t ranges_size = static_cast<uint32_t>(std::ceil((angle_max_ - angle_min_) / angle_increment_));
    scan_msg.ranges.assign(ranges_size, std::numeric_limits<float>::infinity());
    scan_msg.intensities.assign(ranges_size, 0.0);

    for (const auto& point : livox_msg->points)
    {
      // 直接使用雷达原始坐标，不转换
      float x = point.x;
      float y = point.y;
      float z = point.z;

      if (z < min_height_ || z > max_height_) continue;

      float range = std::sqrt(x * x + y * y);
      if (range < range_min_ || range > range_max_) continue;

      float angle = std::atan2(y, x);
      if (angle < angle_min_ || angle > angle_max_) continue;

      int index = static_cast<int>(std::round((angle - angle_min_) / angle_increment_));
      if (index >= 0 && index < static_cast<int>(ranges_size))
      {
        if (range < scan_msg.ranges[index])
        {
          scan_msg.ranges[index] = range;
          scan_msg.intensities[index] = point.reflectivity;
        }
      }
    }

    scan_pub_->publish(scan_msg);
  }

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

  double min_height_, max_height_, angle_min_, angle_max_;
  double angle_increment_, scan_time_, range_min_, range_max_;
  std::string input_topic_, output_topic_;
  int queue_size_, debug_points_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxToScanNode>());
  rclcpp::shutdown();
  return 0;
}
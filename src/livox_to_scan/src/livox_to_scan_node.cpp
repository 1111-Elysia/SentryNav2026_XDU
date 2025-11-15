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
    // 参数
    this->declare_parameter("min_height", -0.5);
    this->declare_parameter("max_height", 2.0);
    this->declare_parameter("angle_min", -M_PI);
    this->declare_parameter("angle_max", M_PI);
    this->declare_parameter("angle_increment", 0.005);
    this->declare_parameter("scan_time", 0.1);
    this->declare_parameter("range_min", 0.1);
    this->declare_parameter("range_max", 100.0);

    min_height_ = this->get_parameter("min_height").as_double();
    max_height_ = this->get_parameter("max_height").as_double();
    angle_min_ = this->get_parameter("angle_min").as_double();
    angle_max_ = this->get_parameter("angle_max").as_double();
    angle_increment_ = this->get_parameter("angle_increment").as_double();
    scan_time_ = this->get_parameter("scan_time").as_double();
    range_min_ = this->get_parameter("range_min").as_double();
    range_max_ = this->get_parameter("range_max").as_double();

    cloud_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        "/livox/lidar", 10,
        std::bind(&LivoxToScanNode::cloudCallback, this, std::placeholders::_1));

    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "Livox → LaserScan 启动");
    RCLCPP_INFO(this->get_logger(), "========================================");
  }

private:
  void cloudCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
  {
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = msg->header.stamp; 
    scan.header.frame_id = "livox_frame"; 

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
      // 保持雷达/车体系：X=右, Y=前, Z=上（不要再旋转）
      float x = pt.x;
      float y = pt.y;
      float z = pt.z;
      if (z < min_height_ || z > max_height_) continue;

      float range = std::sqrt(x*x + y*y);
      if (range < range_min_ || range > range_max_) continue;

      // 角度相对于该帧的 +X（右），逆时针为正（REP-103）
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
  double min_height_, max_height_;
  double angle_min_, angle_max_;
  double angle_increment_, scan_time_;
  double range_min_, range_max_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxToScanNode>());
  rclcpp::shutdown();
  return 0;
}

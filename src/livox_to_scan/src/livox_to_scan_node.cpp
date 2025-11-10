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
    // 声明参数
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
    this->declare_parameter("debug_points", 10); // 每帧打印多少点用于调试

    // 获取参数
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

    // 订阅 Livox CustomMsg 和发布 LaserScan
    cloud_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        input_topic_, queue_size_,
        std::bind(&LivoxToScanNode::cloudCallback, this, std::placeholders::_1));

    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, queue_size_);

    RCLCPP_INFO(this->get_logger(), "Livox to Scan node started");
    RCLCPP_INFO(this->get_logger(), "  Input topic: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Output topic: %s", output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Height filter: [%.2f, %.2f] m", min_height_, max_height_);
    RCLCPP_INFO(this->get_logger(), "  Range filter: [%.2f, %.2f] m", range_min_, range_max_);
    RCLCPP_INFO(this->get_logger(), "  Angle range: [%.2f, %.2f] rad", angle_min_, angle_max_);
  }

private:
  void cloudCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr livox_msg)
  {
    auto scan_msg = sensor_msgs::msg::LaserScan();
    scan_msg.header.stamp = this->get_clock()->now();
    scan_msg.header.frame_id = livox_msg->header.frame_id;
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

    int debug_count = 0;
    int total_points = 0;
    int filtered_height = 0;
    int filtered_range = 0;
    int filtered_angle = 0;
    int accepted_points = 0;

    RCLCPP_INFO_ONCE(this->get_logger(), "Received CustomMsg with %u points", livox_msg->point_num);

    for (const auto& point : livox_msg->points)
    {
      total_points++;

      // CustomPoint 格式：x, y, z 单位已经是米，不需要转换
      float x = point.x;
      float y = point.y;
      float z = point.z;

      // 高度过滤
      if (z < min_height_ || z > max_height_)
      {
        filtered_height++;
        continue;
      }

      // 计算距离和角度
      float range = std::sqrt(x * x + y * y);
      if (range < range_min_ || range > range_max_)
      {
        filtered_range++;
        continue;
      }

      float angle = std::atan2(y, x);
      if (angle < angle_min_ || angle > angle_max_)
      {
        filtered_angle++;
        continue;
      }

      // 计算索引
      int index = static_cast<int>(std::round((angle - angle_min_) / angle_increment_));
      if (index >= 0 && index < static_cast<int>(ranges_size))
      {
        // 保留最近的点
        if (range < scan_msg.ranges[index])
        {
          scan_msg.ranges[index] = range;
          scan_msg.intensities[index] = point.reflectivity;
          accepted_points++;
        }

        // 打印前几个点用于调试
        if (debug_count < debug_points_)
        {
          RCLCPP_INFO(this->get_logger(),
                      "Point[%d]: x=%.3f y=%.3f z=%.3f | angle=%.3f rad (%.1f°) | index=%d range=%.3f reflectivity=%.1f",
                      debug_count, x, y, z, angle, angle * 180.0 / M_PI, index, range, point.reflectivity);
          debug_count++;
        }
      }
    }

    // 统计信息（每100帧打印一次）
    static int frame_count = 0;
    if (++frame_count % 100 == 0)
    {
      RCLCPP_INFO(this->get_logger(),
                  "Stats: total=%d, height_filtered=%d, range_filtered=%d, angle_filtered=%d, accepted=%d",
                  total_points, filtered_height, filtered_range, filtered_angle, accepted_points);
    }

    scan_pub_->publish(scan_msg);
  }

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

  // 参数
  double min_height_;
  double max_height_;
  double angle_min_;
  double angle_max_;
  double angle_increment_;
  double scan_time_;
  double range_min_;
  double range_max_;
  std::string input_topic_;
  std::string output_topic_;
  int queue_size_;
  int debug_points_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxToScanNode>());
  rclcpp::shutdown();
  return 0;
}

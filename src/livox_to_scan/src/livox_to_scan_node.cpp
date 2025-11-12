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
    RCLCPP_INFO(this->get_logger(), "============================================");
    RCLCPP_INFO(this->get_logger(), "  输入话题: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  输出话题: %s", output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  高度过滤: [%.2f, %.2f] m", min_height_, max_height_);
    RCLCPP_INFO(this->get_logger(), "  距离过滤: [%.2f, %.2f] m", range_min_, range_max_);
    RCLCPP_INFO(this->get_logger(), "  角度范围: [%.2f, %.2f] rad", angle_min_, angle_max_);
    RCLCPP_INFO(this->get_logger(), "");
    RCLCPP_WARN(this->get_logger(), "坐标系设定:");
    RCLCPP_WARN(this->get_logger(), "  Livox雷达已是标准ROS坐标系");
    RCLCPP_WARN(this->get_logger(), "  X=前方, Y=左侧, Z=上方");
    RCLCPP_WARN(this->get_logger(), "  不进行坐标转换");
    RCLCPP_INFO(this->get_logger(), "============================================");
  }

private:
  // ===== 坐标系转换函数（不转换）=====
  void transformPointToROS(float x_lidar, float y_lidar, float z_lidar,
                           float& x_ros, float& y_ros, float& z_ros)
  {
    // Livox 雷达已经是标准 ROS 坐标系，不需要转换
    x_ros = x_lidar;   // 前方
    y_ros = y_lidar;   // 左侧
    z_ros = z_lidar;   // 上方
  }

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

    for (const auto& point : livox_msg->points)
    {
      total_points++;

      float x_lidar = point.x;
      float y_lidar = point.y;
      float z_lidar = point.z;

      float x_ros, y_ros, z_ros;
      transformPointToROS(x_lidar, y_lidar, z_lidar, x_ros, y_ros, z_ros);

      // 高度过滤
      if (z_ros < min_height_ || z_ros > max_height_)
      {
        filtered_height++;
        continue;
      }

      // 距离过滤
      float range = std::sqrt(x_ros * x_ros + y_ros * y_ros);
      if (range < range_min_ || range > range_max_)
      {
        filtered_range++;
        continue;
      }

      // 角度计算（ROS标准：0°=X轴正向=前方）
      float angle = std::atan2(y_ros, x_ros);
      
      if (angle < angle_min_ || angle > angle_max_)
      {
        filtered_angle++;
        continue;
      }

      int index = static_cast<int>(std::round((angle - angle_min_) / angle_increment_));
      if (index >= 0 && index < static_cast<int>(ranges_size))
      {
        if (range < scan_msg.ranges[index])
        {
          scan_msg.ranges[index] = range;
          scan_msg.intensities[index] = point.reflectivity;
          accepted_points++;
        }

        if (debug_count < debug_points_)
        {
          RCLCPP_INFO(this->get_logger(),
                      "点[%d]: (%.3f前, %.3f左, %.3f上) | 角度=%.1f° | 距离=%.3fm",
                      debug_count, x_ros, y_ros, z_ros,
                      angle * 180.0 / M_PI, range);
          debug_count++;
        }
      }
    }

    static int frame_count = 0;
    if (++frame_count % 100 == 0)
    {
      RCLCPP_INFO(this->get_logger(),
                  "统计: 总=%d, 高度过滤=%d, 距离过滤=%d, 角度过滤=%d, 接受=%d",
                  total_points, filtered_height, filtered_range, filtered_angle, accepted_points);
      
      int valid_ranges = 0;
      for (const auto& r : scan_msg.ranges) {
        if (std::isfinite(r)) valid_ranges++;
      }
      RCLCPP_INFO(this->get_logger(),
                  "  LaserScan: %d/%d 有效 (%.1f%%)",
                  valid_ranges, ranges_size, 100.0 * valid_ranges / ranges_size);
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
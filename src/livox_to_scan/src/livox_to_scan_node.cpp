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
    this->declare_parameter("debug_points", 10);

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

    // 订阅和发布
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
    RCLCPP_WARN(this->get_logger(), "坐标系转换已启用:");
    RCLCPP_WARN(this->get_logger(), "  雷达坐标系: X=右, Y=前, Z=上");
    RCLCPP_WARN(this->get_logger(), "  ROS 坐标系: X=前, Y=左, Z=上");
    RCLCPP_WARN(this->get_logger(), "  转换公式: X_ros=Y_lidar, Y_ros=-X_lidar");
    RCLCPP_WARN(this->get_logger(), "  角度定义: 0°=前方, +90°=左侧, -90°=右侧");
    RCLCPP_INFO(this->get_logger(), "============================================");
  }

private:
  // ===== 坐标系转换函数 =====
  void transformPointToROS(float x_lidar, float y_lidar, float z_lidar,
                           float& x_ros, float& y_ros, float& z_ros)
  {
    // 雷达坐标系 → ROS 坐标系 (绕Z轴旋转-90度)
    x_ros = y_lidar;   // ROS前方 = 雷达前方
    y_ros = -x_lidar;  // ROS左侧 = -雷达右侧
    z_ros = z_lidar;   // 高度不变
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

    RCLCPP_INFO_ONCE(this->get_logger(), "收到 CustomMsg，包含 %u 个点", livox_msg->point_num);

    for (const auto& point : livox_msg->points)
    {
      total_points++;

      // ===== 关键修改1: 坐标系转换 =====
      // Livox 输出（车体坐标系）
      float x_lidar = point.x;  // 右侧
      float y_lidar = point.y;  // 前方
      float z_lidar = point.z;  // 上方

      // 转换到 ROS 坐标系
      float x_ros, y_ros, z_ros;
      transformPointToROS(x_lidar, y_lidar, z_lidar, x_ros, y_ros, z_ros);

      // ===== 关键修改2: 使用转换后的坐标 =====
      // 高度过滤（使用 ROS 坐标系的 Z）
      if (z_ros < min_height_ || z_ros > max_height_)
      {
        filtered_height++;
        continue;
      }

      // 计算距离（使用 ROS 坐标系的 XY）
      float range = std::sqrt(x_ros * x_ros + y_ros * y_ros);
      if (range < range_min_ || range > range_max_)
      {
        filtered_range++;
        continue;
      }

      // ===== 关键修改3: 计算 ROS 坐标系下的角度 =====
      // 在 ROS 坐标系中: 0° = X轴正方向（前方）
      float angle = std::atan2(y_ros, x_ros);
      
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

        // ===== 调试输出：显示转换前后的坐标 =====
        if (debug_count < debug_points_)
        {
          RCLCPP_INFO(this->get_logger(),
                      "点[%d] 转换:\n"
                      "  雷达系: (%.3f右, %.3f前, %.3f上)\n"
                      "  ROS系:  (%.3f前, %.3f左, %.3f上)\n"
                      "  角度=%.1f° | 距离=%.3fm | 索引=%d | 强度=%.0f",
                      debug_count,
                      x_lidar, y_lidar, z_lidar,
                      x_ros, y_ros, z_ros,
                      angle * 180.0 / M_PI, range, index, point.reflectivity);
          debug_count++;
        }
      }
    }

    // 统计信息（每100帧打印一次）
    static int frame_count = 0;
    if (++frame_count % 100 == 0)
    {
      RCLCPP_INFO(this->get_logger(),
                  "统计: 总点数=%d, 高度过滤=%d, 距离过滤=%d, 角度过滤=%d, 接受=%d",
                  total_points, filtered_height, filtered_range, filtered_angle, accepted_points);
      
      // 额外输出：有效点的角度分布
      int valid_ranges = 0;
      for (const auto& r : scan_msg.ranges) {
        if (std::isfinite(r)) valid_ranges++;
      }
      RCLCPP_INFO(this->get_logger(),
                  "  LaserScan: %d/%d 个有效距离值 (%.1f%%)",
                  valid_ranges, ranges_size, 100.0 * valid_ranges / ranges_size);
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
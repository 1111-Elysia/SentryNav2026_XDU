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

    min_height_ = this->get_parameter("min_height").as_double();
    max_height_ = this->get_parameter("max_height").as_double();
    angle_min_ = this->get_parameter("angle_min").as_double();
    angle_max_ = this->get_parameter("angle_max").as_double();
    angle_increment_ = this->get_parameter("angle_increment").as_double();
    scan_time_ = this->get_parameter("scan_time").as_double();
    range_min_ = this->get_parameter("range_min").as_double();
    range_max_ = this->get_parameter("range_max").as_double();

    cloud_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        "/livox/lidar", 50,
        std::bind(&LivoxToScanNode::cloudCallback, this, std::placeholders::_1));

    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 50);

    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "Livox to Scan 启动");
    RCLCPP_INFO(this->get_logger(), "  雷达系(X=右,Y=前) → ROS系(X=前,Y=左)");
    RCLCPP_INFO(this->get_logger(), "  转换: 旋转-90度 + 翻转180度");
    RCLCPP_INFO(this->get_logger(), "  frame_id: base_link");
    RCLCPP_INFO(this->get_logger(), "========================================");
  }

private:
  void cloudCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
  {
    auto scan = sensor_msgs::msg::LaserScan();
    scan.header.stamp = this->get_clock()->now();
    
    // ===== frame_id 必须是 base_link =====
    scan.header.frame_id = "base_link";
    
    scan.angle_min = angle_min_;
    scan.angle_max = angle_max_;
    scan.angle_increment = angle_increment_;
    scan.time_increment = 0.0;
    scan.scan_time = scan_time_;
    scan.range_min = range_min_;
    scan.range_max = range_max_;

    uint32_t size = std::ceil((angle_max_ - angle_min_) / angle_increment_);
    scan.ranges.assign(size, std::numeric_limits<float>::infinity());
    scan.intensities.assign(size, 0.0);

    for (const auto& pt : msg->points)
    {
      // 雷达坐标系: X=右, Y=前
      float x_lidar = pt.x;
      float y_lidar = pt.y;
      float z_lidar = pt.z;

      if (z_lidar < min_height_ || z_lidar > max_height_) continue;

      // ===== 完整转换到 base_link (ROS系: X=前, Y=左) =====
      // 步骤1: 旋转180度（因为雷达坐标系反了）
      float x_flipped = -x_lidar;
      float y_flipped = -y_lidar;
      
      // 步骤2: 旋转-90度 (车体系 → ROS系)
      // [X_ros]   [ 0  1] [X_flipped]   [ Y_flipped]
      // [Y_ros] = [-1  0] [Y_flipped] = [-X_flipped]
      float x_ros = y_flipped;
      float y_ros = -x_flipped;

      float range = std::sqrt(x_ros*x_ros + y_ros*y_ros);
      if (range < range_min_ || range > range_max_) continue;

      // 在ROS坐标系下计算角度
      float angle = std::atan2(y_ros, x_ros);
      if (angle < angle_min_ || angle > angle_max_) continue;

      int idx = std::round((angle - angle_min_) / angle_increment_);
      if (idx >= 0 && idx < (int)size)
      {
        if (range < scan.ranges[idx])
        {
          scan.ranges[idx] = range;
          scan.intensities[idx] = pt.reflectivity;
        }
      }
    }

    scan_pub_->publish(scan);
  }

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  double min_height_, max_height_, angle_min_, angle_max_;
  double angle_increment_, scan_time_, range_min_, range_max_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxToScanNode>());
  rclcpp::shutdown();
  return 0;
}
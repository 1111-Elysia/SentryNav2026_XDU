#include "costmap_voxel/voxel_filter_layer.hpp"

#include <algorithm>
#include <limits>

#include "pluginlib/class_list_macros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/filters/voxel_grid.h>

namespace costmap_voxel {

VoxelFilterLayer::VoxelFilterLayer()
: nav2_costmap_2d::Layer()
{
  cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
}

void VoxelFilterLayer::onInitialize()
{
  auto node = node_.lock();
  logger_ = node->get_logger();

  // 读取参数（使用 layer 名称前缀）
  std::string ns = name_ + ".";
  node->declare_parameter<bool>(ns + "enabled", true);
  node->get_parameter(ns + "enabled", enabled_);

  node->declare_parameter<double>(ns + "voxel_leaf_size", 0.1);
  node->get_parameter(ns + "voxel_leaf_size", voxel_leaf_size_);

  node->declare_parameter<double>(ns + "max_obstacle_intensity", 2.0);
  node->get_parameter(ns + "max_obstacle_intensity", max_obstacle_intensity_);

  node->declare_parameter<bool>(ns + "pointcloud.marking", true);
  node->get_parameter(ns + "pointcloud.marking", marking_);

  node->declare_parameter<bool>(ns + "pointcloud.clearing", true);
  node->get_parameter(ns + "pointcloud.clearing", clearing_);

  node->declare_parameter<std::string>(ns + "pointcloud.topic", "/livox/lidar/pointcloud");
  node->get_parameter(ns + "pointcloud.topic", topic_);

  RCLCPP_INFO(logger_, "[%s] enabled=%d leaf=%.3f max_intensity=%.3f topic=%s",
              name_.c_str(), enabled_, voxel_leaf_size_, max_obstacle_intensity_, topic_.c_str());

  // 订阅点云
  sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    topic_, rclcpp::SensorDataQoS(),
    std::bind(&VoxelFilterLayer::cloudCallback, this, std::placeholders::_1));

  current_ = true;
}

void VoxelFilterLayer::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!enabled_) return;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*msg, *cloud_in);

  // 体素滤波
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  vg.setInputCloud(cloud_in);
  vg.setLeafSize(static_cast<float>(voxel_leaf_size_),
                 static_cast<float>(voxel_leaf_size_),
                 static_cast<float>(voxel_leaf_size_));
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ds(new pcl::PointCloud<pcl::PointXYZI>());
  vg.filter(*cloud_ds);

  // 按强度阈值筛除（大于阈值忽略）
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());
  cloud_filtered->reserve(cloud_ds->size());
  for (const auto& p : cloud_ds->points) {
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
        p.intensity <= max_obstacle_intensity_) {
      cloud_filtered->push_back(p);
    }
  }

  {
    std::lock_guard<std::mutex> lk(mutex_);
    *cloud_ = std::move(*cloud_filtered);
  }

  // 通知 costmap 更新
  current_ = false;
}

void VoxelFilterLayer::updateBounds(double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
                                    double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (!enabled_) return;

  double lx = std::numeric_limits<double>::max();
  double ly = std::numeric_limits<double>::max();
  double hx = -std::numeric_limits<double>::max();
  double hy = -std::numeric_limits<double>::max();

  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (cloud_->empty()) return;

    for (const auto& p : cloud_->points) {
      lx = std::min(lx, static_cast<double>(p.x));
      ly = std::min(ly, static_cast<double>(p.y));
      hx = std::max(hx, static_cast<double>(p.x));
      hy = std::max(hy, static_cast<double>(p.y));
    }
  }

  last_min_x_ = lx; last_min_y_ = ly; last_max_x_ = hx; last_max_y_ = hy;
  *min_x = std::min(*min_x, lx);
  *min_y = std::min(*min_y, ly);
  *max_x = std::max(*max_x, hx);
  *max_y = std::max(*max_y, hy);
}

void VoxelFilterLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                                   int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || !marking_) return;

  // 将过滤后的点标记为障碍物
  {
    std::lock_guard<std::mutex> lk(mutex_);
    for (const auto& p : cloud_->points) {
      unsigned int mx, my;
      if (master_grid.worldToMap(p.x, p.y, mx, my)) {
        if (static_cast<int>(mx) >= min_i && static_cast<int>(mx) < max_i &&
            static_cast<int>(my) >= min_j && static_cast<int>(my) < max_j) {
          master_grid.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
        }
      }
    }
  }

  current_ = true;
}

} // namespace costmap_voxel

PLUGINLIB_EXPORT_CLASS(costmap_voxel::VoxelFilterLayer, nav2_costmap_2d::Layer)

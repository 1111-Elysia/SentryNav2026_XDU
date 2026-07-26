#pragma once

#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace costmap_voxel {

class VoxelFilterLayer : public nav2_costmap_2d::Layer {
public:
  VoxelFilterLayer();
  ~VoxelFilterLayer() override = default;

  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y, double* max_x, double* max_y) override;
  void updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                   int min_i, int min_j, int max_i, int max_j) override;
  bool isClearable() override { return false; } // 本层不清理
  void reset() override {}

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;
  std::mutex mutex_;

  // params
  double voxel_leaf_size_{0.1};
  double max_obstacle_intensity_{2.0};
  bool marking_{true};
  bool clearing_{true};
  std::string topic_;

  // cached bounds from last update
  double last_min_x_{0.0}, last_min_y_{0.0}, last_max_x_{0.0}, last_max_y_{0.0};

  rclcpp::Logger logger_{rclcpp::get_logger("VoxelFilterLayer")};
};

} // namespace costmap_voxel

// line_crossing_layer.hpp
// 代价地图插件：在线段两侧创建代价带，引导路径垂直穿越直线
//
// 原理:
//   1. 读取与 pri_adaptive_mppi 相同格式的直线定义
//   2. 在每条直线两侧 band_width 范围内标记代价
//   3. A* 规划器自然选择最短（最垂直）的穿越路径
//
// 参数 (均以 name_ + "." 为前缀):
//   enabled           (bool)         - 是否启用
//   band_width        (double, 0.5)  - 单侧代价带宽度 (m)
//   cost_value        (int, 200)     - 代价带内代价值 (0-255)
//   corner_radius         (double, 0.3)  - 端点拐角代价圆半径 (m), 0=禁用
//   corner_cost_value     (int, 253)     - 拐角中心峰值代价
//   corner_gradient_power (double, 1.0)  - 梯度衰减指数 (1=线性, 2=二次, 0.5=凹)
//   lines             (string[])     - 直线名称列表
//   <line>.point_1    (double[2])    - 端点1坐标
//   <line>.point_2    (double[2])    - 端点2坐标

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_ros/buffer.h"

namespace line_crossing_layer
{

/// 直线定义（轻量版，仅几何信息）
struct LineDef
{
  std::string name;

  double x1 = 0.0, y1 = 0.0;
  double x2 = 5.0, y2 = 0.0;

  // 预计算
  double dx = 0.0, dy = 0.0, length = 0.0;
};

/// 直线穿越代价地图层
///
/// 在每条直线的两侧创建对称代价带。路径穿过代价带时：
///   - 垂直穿越 = 2 * band_width 距离 → 代价最小
///   - 浅角度穿越 = 更长距离 → 代价更大
///   - 端点处额外圆形代价区 → 阻止从拐角穿越
/// A* 自然偏好垂直穿越。
class LineCrossingLayer : public nav2_costmap_2d::Layer
{
public:
  LineCrossingLayer() = default;
  ~LineCrossingLayer() override = default;

  // ── nav2_costmap_2d::Layer 纯虚函数 ──
  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void matchSize() override;
  void reset() override;
  bool isClearable() override { return false; }

private:
  /// 从线段列表中提取去重后的唯一点集合
  void collectUniquePoints();

  std::vector<LineDef> lines_;
  double band_width_{0.5};
  unsigned char cost_value_{200};

  double corner_radius_{0.3};
  unsigned char corner_cost_value_{253};
  double corner_gradient_power_{1.0};  // 1=linear, 2=quadratic, 0.5=concave
  /// 去重后的唯一点 (x,y)，来自所有线段的端点
  std::vector<std::pair<double, double>> corner_points_;
};

}  // namespace line_crossing_layer

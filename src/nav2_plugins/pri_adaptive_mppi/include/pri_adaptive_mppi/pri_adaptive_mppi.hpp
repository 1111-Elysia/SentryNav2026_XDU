// pri_adaptive_mppi.hpp
// 自适应MPPI控制器wrapper — 根据路径穿越直线方向自动切换三套MPPI参数
//
// 原理:
//   1. 用pluginlib加载内部 nav2_mppi_controller::MPPIController 实例
//   2. 支持多条直线，每条直线有独立几何参数和上坡/下坡MPPI参数
//   3. 在computeVelocityCommands中检测规划路径是否穿越任意直线
//   4. 若穿越，判断方向（上坡→下坡 或 下坡→上坡），切换对应直线的MPPI参数集
//   5. 多条直线同时进入膨胀层时，选距离机器人最近的那条
//   6. 全局 normal.* 参数用于未穿越任何直线时

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker.hpp"

namespace pri_adaptive_mppi
{

// ═══════════════════════════════════════════════════════════
//  模式枚举
// ═══════════════════════════════════════════════════════════

enum class CrossingMode
{
  NORMAL   = 0,  // 未进入膨胀区 / 未相交 — 使用 normal.* 参数
  UPHILL   = 1,  // 上坡→下坡穿越（左侧→右侧） — 使用 <line>.uphill.* 参数
  DOWNHILL = 2   // 下坡→上坡穿越（右侧→左侧） — 使用 <line>.downhill.* 参数
};

// ═══════════════════════════════════════════════════════════
//  几何工具函数
// ═══════════════════════════════════════════════════════════

/// 计算二维叉积 z 分量: cross(AB, AC) = (B-A) x (C-A)
inline double cross2d(
  double ax, double ay, double bx, double by, double cx, double cy)
{
  return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

/// 计算点 P 到线段 P1-P2 的有符号距离（左侧为正）
inline double signedDistanceToLine(
  double px, double py,
  double lx1, double ly1, double lx2, double ly2)
{
  double line_dx = lx2 - lx1;
  double line_dy = ly2 - ly1;
  return (line_dx * (py - ly1) - line_dy * (px - lx1));
}

/// 计算点 P 到线段 P1-P2 的垂直距离（绝对值）
inline double perpendicularDistanceToLine(
  double px, double py,
  double lx1, double ly1, double lx2, double ly2,
  double line_length)
{
  if (line_length < 1e-9) {
    return std::hypot(px - lx1, py - ly1);
  }
  return std::abs(signedDistanceToLine(px, py, lx1, ly1, lx2, ly2)) / line_length;
}

/// 判断线段 AB 与线段 CD 是否严格相交（不包括端点接触）
inline bool segmentsIntersect(
  double ax, double ay, double bx, double by,
  double cx, double cy, double dx, double dy)
{
  double d1 = cross2d(cx, cy, dx, dy, ax, ay);  // CD x CA
  double d2 = cross2d(cx, cy, dx, dy, bx, by);  // CD x CB
  double d3 = cross2d(ax, ay, bx, by, cx, cy);  // AB x AC
  double d4 = cross2d(ax, ay, bx, by, dx, dy);  // AB x AD

  // 严格跨立：两端点必须在直线异侧（乘积 < 0）
  return (d1 * d2 < 0.0) && (d3 * d4 < 0.0);
}

/// 检测路径是否穿越直线，返回穿越方向
/// 约定: 直线方向(point1→point2)的左侧 = 上坡侧, 右侧 = 下坡侧
/// @return  0 = 无穿越
///         +1 = 上坡→下坡 (左侧→右侧)
///         -1 = 下坡→上坡 (右侧→左侧)
inline int detectPathCrossing(
  const nav_msgs::msg::Path & path,
  double lx1, double ly1, double lx2, double ly2)
{
  if (path.poses.size() < 2) {
    return 0;
  }

  for (size_t i = 0; i < path.poses.size() - 1; ++i) {
    double ax = path.poses[i].pose.position.x;
    double ay = path.poses[i].pose.position.y;
    double bx = path.poses[i + 1].pose.position.x;
    double by = path.poses[i + 1].pose.position.y;

    if (segmentsIntersect(ax, ay, bx, by, lx1, ly1, lx2, ly2)) {
      double signed_a = signedDistanceToLine(ax, ay, lx1, ly1, lx2, ly2);

      if (signed_a > 0.0) {
        return +1;  // A在上坡侧(左) → 上坡→下坡
      } else if (signed_a < 0.0) {
        return -1;  // A在下坡侧(右) → 下坡→上坡
      }
    }
  }
  return 0;
}

// ═══════════════════════════════════════════════════════
//  直线配置结构体
// ═══════════════════════════════════════════════════════

struct LineConfig
{
  std::string name;

  // ── 几何参数 ──
  double x1 = 0.0, y1 = 0.0;
  double x2 = 5.0, y2 = 0.0;
  double inflation_radius_uphill = 1.0;    // 上坡侧(左)膨胀半径
  double inflation_radius_downhill = 1.0;  // 下坡侧(右)膨胀半径

  // ── 预计算 ──
  double dx = 0.0, dy = 0.0, length = 0.0;

  // ── 可视化 Marker ──
  visualization_msgs::msg::Marker line_marker;
  visualization_msgs::msg::Marker uphill_zone_marker;
  visualization_msgs::msg::Marker downhill_zone_marker;

  /// 构建膨胀区边界点 (LINE_LIST)
  /// @param offset_sign  +1 = 上坡侧(左), -1 = 下坡侧(右)
  /// @param radius      当前侧的膨胀半径
  std::vector<geometry_msgs::msg::Point> buildZoneLines(double offset_sign, double radius) const
  {
    double nx, ny;
    if (length < 1e-9) {
      nx = 0.0; ny = 1.0;
    } else {
      nx = -dy / length;
      ny =  dx / length;
    }

    double ox = offset_sign * radius * nx;
    double oy = offset_sign * radius * ny;

    geometry_msgs::msg::Point a, b, c, d;
    a.x = x1;       a.y = y1;       a.z = 0.0;
    b.x = x1 + ox;  b.y = y1 + oy;  b.z = 0.0;
    c.x = x2 + ox;  c.y = y2 + oy;  c.z = 0.0;
    d.x = x2;       d.y = y2;       d.z = 0.0;

    return {a, b, b, c, c, d, d, a};
  }
};

// ═══════════════════════════════════════════════════════════
//  主类: PriAdaptiveMppi
// ═══════════════════════════════════════════════════════════

class PriAdaptiveMppi : public nav2_core::Controller
{
public:
  PriAdaptiveMppi() = default;
  ~PriAdaptiveMppi() override = default;

  // ── Nav2 Controller 接口 ──
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  // ── 内部方法 ──

  /// 声明插件专属参数
  void declareCommonParams();

  /// 初始化每条直线的可视化 Marker
  void initLineMarkers(LineConfig & line, int ns_suffix);

  /// 根据模式和直线索引切换内部 MPPI 参数
  void switchMode(int line_index, CrossingMode new_mode);

  /// 获取模式对应的参数名前缀
  static const char * modeToPrefix(CrossingMode mode);

  /// 发布所有直线的可视化 Marker
  void publishVisualization();

  // ── pluginlib ──
  pluginlib::UniquePtr<nav2_core::Controller> inner_controller_;
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::Controller>> loader_;
  std::string inner_plugin_type_;

  // ── ROS2 资源 ──
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("PriAdaptiveMppi")};
  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  // ── 可视化 ──
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_pub_;

  // ── 直线配置 ──
  std::vector<LineConfig> lines_;
  double max_path_age_{2.0};

  // ── 当前状态 ──
  int active_line_index_{-1};        // -1 = 无活跃直线（使用 normal）
  CrossingMode active_crossing_{CrossingMode::NORMAL};
  nav_msgs::msg::Path current_path_;
  rclcpp::Time last_path_time_{0, 0, RCL_ROS_TIME};
  bool path_received_{false};

  // ── 模式前缀常量 ──
  static constexpr const char * PREFIX_NORMAL   = "normal";
  static constexpr const char * PREFIX_UPHILL   = "uphill";
  static constexpr const char * PREFIX_DOWNHILL = "downhill";
};

}  // namespace pri_adaptive_mppi

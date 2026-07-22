// teammate_cost_layer.hpp
// 代价地图插件：在队友机器人坐标周围创建圆形渐变代价区域
//
// 原理:
//   1. 订阅 /ground_pos_relay/teammate_pos_odom（teammate_frame_converter 的输出）
//   2. 在每个启用的队友位置周围创建两层代价区域：
//      - 核心区 (dist ≤ core_radius):          固定代价值 (默认 LETHAL=254)
//      - 衰减区 (core_radius < dist ≤ total):  幂梯度衰减到 0
//   3. MPPI 控制器感知梯度，自然绕开队友
//
// 参数 (以 name_ + "." 为前缀):
//   enabled              (bool)           - 是否启用
//   teammate_topic       (string)         - 队友位置话题
//   enable_hero          (bool, true)     - 对英雄添加代价
//   enable_engineer      (bool, true)     - 对工程添加代价
//   enable_standard_3    (bool, false)    - 对步兵3添加代价
//   enable_standard_4    (bool, false)    - 对步兵4添加代价
//   enable_sentry        (bool, true)     - 对哨兵添加代价
//   core_radius          (double, 0.5)    - 核心致命区半径 (m)
//   decay_radius         (double, 0.5)    - 衰减区宽度 (m)，总半径 = core + decay
//   core_cost_value      (int, 254)       - 核心区代价值 (0-255)
//   gradient_power       (double, 1.5)    - 衰减指数 (1=线性, 2=二次)
//   teammate_timeout     (double, 2.0)    - 队友数据超时 (s), 超时后清除代价

#pragma once

#include <memory>
#include <string>
#include <mutex>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rm_referee_msgs/msg/ground_robot_position.hpp"

namespace teammate_cost_layer
{

/// 圆形渐变代价地图层
///
/// 在每个启用的队友坐标周围创建圆形代价区域，让导航规划器
/// 自然避开友方单位。
///
/// 代价模型:
///   - 核心区 (d ≤ core_radius):            cost = core_cost_value
///   - 衰减区 (core < d ≤ core+decay):      cost = core_cost_value × (1 - t)^power, t = (d-core)/decay
///   - 区域外:                               无代价
class TeammateCostLayer : public nav2_costmap_2d::Layer
{
public:
  TeammateCostLayer() = default;
  ~TeammateCostLayer() override = default;

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
  /// 队友位置订阅回调
  void teammateCallback(
    const rm_referee_msgs::msg::GroundRobotPosition::SharedPtr msg);

  /// 对单个队友位置应用代价
  void applyTeammateCost(
    nav2_costmap_2d::Costmap2D & master_grid,
    double tx, double ty);

  /// 判断队友数据是否有效（未超时）
  /// @pre 调用者必须持有 positions_mutex_
  bool isDataValid() const;

  // ── 队友位置缓存 ──
  struct TeammatePositions
  {
    double hero_x{0.0}, hero_y{0.0};
    double engineer_x{0.0}, engineer_y{0.0};
    double standard_3_x{0.0}, standard_3_y{0.0};
    double standard_4_x{0.0}, standard_4_y{0.0};
    double sentry_x{0.0}, sentry_y{0.0};
    rclcpp::Time stamp;
    bool valid{false};
  };

  TeammatePositions positions_;
  mutable std::mutex positions_mutex_;

  // ── 订阅 ──
  rclcpp::Subscription<rm_referee_msgs::msg::GroundRobotPosition>::SharedPtr teammate_sub_;

  // ── 日志 ──
  rclcpp::Logger logger_{rclcpp::get_logger("teammate_cost_layer")};

  // ── 参数 ──
  bool enable_hero_{true};
  bool enable_engineer_{true};
  bool enable_standard_3_{false};
  bool enable_standard_4_{false};
  bool enable_sentry_{true};

  double core_radius_{0.5};
  double decay_radius_{0.5};
  unsigned char core_cost_value_{254};
  double gradient_power_{1.5};
  double teammate_timeout_{2.0};
};

}  // namespace teammate_cost_layer

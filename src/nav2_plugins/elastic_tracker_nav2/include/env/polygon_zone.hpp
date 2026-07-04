/*
    PolygonZone: 多边形区域判定 + RViz 显示
    用于 ElasticTracker 的目标有效性门控
*/
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>

#include <string>
#include <vector>
#include <memory>

namespace elastic_planner {

class PolygonZone {
public:
  PolygonZone() = default;

  /// 从参数数组加载顶点: [x1,y1, x2,y2, x3,y3, ...]
  void loadVertices(const std::vector<double> & flat_vertices);

  /// 射线投射判定点是否在多边形内
  bool isInside(double x, double y) const;

  bool enabled() const { return !vertices_.empty(); }

  /// 激活 RViz PolygonStamped 定时发布
  void activatePublisher(rclcpp_lifecycle::LifecycleNode * node, const std::string & topic = "/tracking_zone");

private:
  std::vector<geometry_msgs::msg::Point32> vertices_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
};

// ==================== inline impl ====================

inline void PolygonZone::loadVertices(const std::vector<double> & flat) {
  vertices_.clear();
  for (size_t i = 0; i + 1 < flat.size(); i += 2) {
    geometry_msgs::msg::Point32 pt;
    pt.x = static_cast<float>(flat[i]);
    pt.y = static_cast<float>(flat[i + 1]);
    pt.z = 0.0f;
    vertices_.push_back(pt);
  }
}

inline bool PolygonZone::isInside(double x, double y) const {
  if (vertices_.size() < 3) return true;  // 无顶点或边不够 → 不限
  int count = 0;
  size_t n = vertices_.size();
  for (size_t i = 0; i < n; i++) {
    const auto &a = vertices_[i];
    const auto &b = vertices_[(i + 1) % n];
    if ((a.y > y) != (b.y > y) &&
        x < (b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x)
      count++;
  }
  return count & 1;  // 奇数 = 内部
}

inline void PolygonZone::activatePublisher(rclcpp_lifecycle::LifecycleNode * node, const std::string & topic) {
  if (vertices_.empty()) return;
  pub_ = node->create_publisher<geometry_msgs::msg::PolygonStamped>(topic, rclcpp::QoS(1).transient_local());
  // 1Hz 定时发布
  pub_timer_ = node->create_wall_timer(
    std::chrono::seconds(1),
    [this]() {
      if (!pub_ || vertices_.empty()) return;
      geometry_msgs::msg::PolygonStamped msg;
      msg.header.frame_id = "map";
      msg.header.stamp = rclcpp::Clock().now();
      msg.polygon.points = vertices_;
      pub_->publish(msg);
    });
}

}  // namespace elastic_planner

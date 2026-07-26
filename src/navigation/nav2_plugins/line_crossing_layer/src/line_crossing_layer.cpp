// line_crossing_layer.cpp — 实现

#include "line_crossing_layer/line_crossing_layer.hpp"

namespace line_crossing_layer
{

void LineCrossingLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("LineCrossingLayer: Failed to lock node in onInitialize()");
  }

  logger_ = rclcpp::get_logger("line_crossing_layer." + name_);

  // ── 声明并读取参数 ──
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".enabled", rclcpp::ParameterValue(true));
  enabled_ = node->get_parameter(name_ + ".enabled").as_bool();

  if (!enabled_) {
    RCLCPP_INFO(logger_, "LineCrossingLayer '%s' is disabled.", name_.c_str());
    current_ = true;
    return;
  }

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".band_width", rclcpp::ParameterValue(0.5));
  band_width_ = node->get_parameter(name_ + ".band_width").as_double();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".cost_value", rclcpp::ParameterValue(200));
  int cost_int = node->get_parameter(name_ + ".cost_value").as_int();
  cost_value_ = static_cast<unsigned char>(std::clamp(cost_int, 0, 255));

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".corner_radius", rclcpp::ParameterValue(0.3));
  corner_radius_ = node->get_parameter(name_ + ".corner_radius").as_double();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".corner_cost_value", rclcpp::ParameterValue(253));
  int corner_int = node->get_parameter(name_ + ".corner_cost_value").as_int();
  corner_cost_value_ = static_cast<unsigned char>(std::clamp(corner_int, 0, 255));

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".corner_gradient_power", rclcpp::ParameterValue(1.0));
  corner_gradient_power_ = node->get_parameter(name_ + ".corner_gradient_power").as_double();

  // ── 读取直线列表 ──
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lines", rclcpp::ParameterValue(std::vector<std::string>{}));
  std::vector<std::string> line_names;
  node->get_parameter(name_ + ".lines", line_names);

  // ── 解析每条直线的几何参数 ──
  for (const auto & ln : line_names) {
    LineDef line;
    line.name = ln;

    std::string p1_key = name_ + "." + ln + ".point_1";
    std::string p2_key = name_ + "." + ln + ".point_2";

    nav2_util::declare_parameter_if_not_declared(
      node, p1_key, rclcpp::ParameterValue(std::vector<double>{0.0, 0.0}));
    nav2_util::declare_parameter_if_not_declared(
      node, p2_key, rclcpp::ParameterValue(std::vector<double>{5.0, 0.0}));

    std::vector<double> p1, p2;
    node->get_parameter(p1_key, p1);
    node->get_parameter(p2_key, p2);

    if (p1.size() < 2 || p2.size() < 2) {
      RCLCPP_WARN(logger_,
        "Line '%s': point_1 or point_2 has fewer than 2 elements, skipping.", ln.c_str());
      continue;
    }

    line.x1 = p1[0]; line.y1 = p1[1];
    line.x2 = p2[0]; line.y2 = p2[1];

    // 预计算
    line.dx = line.x2 - line.x1;
    line.dy = line.y2 - line.y1;
    line.length = std::hypot(line.dx, line.dy);

    lines_.push_back(line);
  }

  // ── 提取去重后的唯一点（用于拐角代价） ──
  collectUniquePoints();

  RCLCPP_INFO(logger_,
    "LineCrossingLayer '%s' initialized: %zu lines, band=%.2fm cost=%d, "
    "corner_r=%.2fm corner_cost=%d, %zu unique corners",
    name_.c_str(), lines_.size(), band_width_, static_cast<int>(cost_value_),
    corner_radius_, static_cast<int>(corner_cost_value_), corner_points_.size());

  current_ = true;
  matchSize();
}

void LineCrossingLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_ || lines_.empty()) {
    return;
  }

  for (const auto & line : lines_) {
    *min_x = std::min(*min_x, std::min(line.x1, line.x2) - band_width_);
    *min_y = std::min(*min_y, std::min(line.y1, line.y2) - band_width_);
    *max_x = std::max(*max_x, std::max(line.x1, line.x2) + band_width_);
    *max_y = std::max(*max_y, std::max(line.y1, line.y2) + band_width_);
  }

  // 扩展拐角代价区域
  if (corner_radius_ > 0.0) {
    for (const auto & pt : corner_points_) {
      *min_x = std::min(*min_x, pt.first  - corner_radius_);
      *min_y = std::min(*min_y, pt.second - corner_radius_);
      *max_x = std::max(*max_x, pt.first  + corner_radius_);
      *max_y = std::max(*max_y, pt.second + corner_radius_);
    }
  }
}

void LineCrossingLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || lines_.empty()) {
    return;
  }

  unsigned int size_x = master_grid.getSizeInCellsX();
  unsigned int size_y = master_grid.getSizeInCellsY();

  // 裁剪到有效范围
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);

  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      unsigned int index = master_grid.getIndex(i, j);

      // 获取 cell 中心的世界坐标
      double wx, wy;
      master_grid.mapToWorld(i, j, wx, wy);

      for (const auto & line : lines_) {
        // ── 快速剔除：AABB 检测 ──
        double line_min_x = std::min(line.x1, line.x2) - band_width_;
        double line_max_x = std::max(line.x1, line.x2) + band_width_;
        double line_min_y = std::min(line.y1, line.y2) - band_width_;
        double line_max_y = std::max(line.y1, line.y2) + band_width_;

        if (wx < line_min_x || wx > line_max_x ||
            wy < line_min_y || wy > line_max_y) {
          continue;
        }

        // ── 计算点到线段的最短距离 ──
        double t;
        if (line.length < 1e-9) {
          t = 0.0;
        } else {
          t = ((wx - line.x1) * line.dx + (wy - line.y1) * line.dy) /
              (line.length * line.length);
        }
        t = std::clamp(t, 0.0, 1.0);

        double nearest_x = line.x1 + t * line.dx;
        double nearest_y = line.y1 + t * line.dy;
        double dist = std::hypot(wx - nearest_x, wy - nearest_y);

        if (dist <= band_width_) {
          unsigned char current_cost = master_grid.getCost(index);
          if (cost_value_ > current_cost) {
            master_grid.setCost(i, j, cost_value_);
          }
          break;  // 命中一条直线即可，无需检查其他
        }
      }

      // ── 拐角代价：径向线性梯度（中心最高，边缘渐降到 0） ──
      if (corner_radius_ > 0.0 && !corner_points_.empty()) {
        for (const auto & pt : corner_points_) {
          double dx = wx - pt.first;
          double dy = wy - pt.second;
          double dist = std::hypot(dx, dy);
          if (dist < corner_radius_) {
            // 幂梯度: cost = peak × (1 - dist/r)^power
            unsigned char gradient_cost = static_cast<unsigned char>(
              corner_cost_value_ *
              std::pow(1.0 - dist / corner_radius_, corner_gradient_power_));
            unsigned char current_cost = master_grid.getCost(index);
            if (gradient_cost > current_cost) {
              master_grid.setCost(i, j, gradient_cost);
            }
            break;
          }
        }
      }
    }
  }

  current_ = true;
}

void LineCrossingLayer::matchSize()
{
  // 本层没有内部数组需要调整大小
}

void LineCrossingLayer::reset()
{
  current_ = false;
}

void LineCrossingLayer::collectUniquePoints()
{
  corner_points_.clear();
  if (corner_radius_ <= 0.0) {
    return;
  }

  constexpr double kEps = 1e-3;  // 1mm 容差，合并端点
  auto is_close = [](double x1, double y1, double x2, double y2) {
    return std::hypot(x1 - x2, y1 - y2) < kEps;
  };

  for (const auto & line : lines_) {
    // 检查 point_1
    bool found1 = false;
    for (const auto & pt : corner_points_) {
      if (is_close(line.x1, line.y1, pt.first, pt.second)) {
        found1 = true;
        break;
      }
    }
    if (!found1) {
      corner_points_.emplace_back(line.x1, line.y1);
    }

    // 检查 point_2
    bool found2 = false;
    for (const auto & pt : corner_points_) {
      if (is_close(line.x2, line.y2, pt.first, pt.second)) {
        found2 = true;
        break;
      }
    }
    if (!found2) {
      corner_points_.emplace_back(line.x2, line.y2);
    }
  }
}

}  // namespace line_crossing_layer

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  line_crossing_layer::LineCrossingLayer, nav2_costmap_2d::Layer)

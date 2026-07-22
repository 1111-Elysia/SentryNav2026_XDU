// teammate_cost_layer.cpp — 实现

#include "teammate_cost_layer/teammate_cost_layer.hpp"

namespace teammate_cost_layer
{

void TeammateCostLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("TeammateCostLayer: Failed to lock node in onInitialize()");
  }

  logger_ = rclcpp::get_logger("teammate_cost_layer." + name_);

  // ── 声明并读取参数 ──
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".enabled", rclcpp::ParameterValue(true));
  enabled_ = node->get_parameter(name_ + ".enabled").as_bool();

  if (!enabled_) {
    RCLCPP_INFO(logger_, "TeammateCostLayer '%s' is disabled.", name_.c_str());
    current_ = true;
    return;
  }

  // 话题
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".teammate_topic",
    rclcpp::ParameterValue(std::string("/ground_pos_relay/teammate_pos_odom")));
  std::string topic = node->get_parameter(name_ + ".teammate_topic").as_string();

  // 队友选择
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".enable_hero", rclcpp::ParameterValue(true));
  enable_hero_ = node->get_parameter(name_ + ".enable_hero").as_bool();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".enable_engineer", rclcpp::ParameterValue(true));
  enable_engineer_ = node->get_parameter(name_ + ".enable_engineer").as_bool();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".enable_standard_3", rclcpp::ParameterValue(false));
  enable_standard_3_ = node->get_parameter(name_ + ".enable_standard_3").as_bool();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".enable_standard_4", rclcpp::ParameterValue(false));
  enable_standard_4_ = node->get_parameter(name_ + ".enable_standard_4").as_bool();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".enable_sentry", rclcpp::ParameterValue(true));
  enable_sentry_ = node->get_parameter(name_ + ".enable_sentry").as_bool();

  // 代价区域参数
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".core_radius", rclcpp::ParameterValue(0.5));
  core_radius_ = node->get_parameter(name_ + ".core_radius").as_double();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".decay_radius", rclcpp::ParameterValue(0.5));
  decay_radius_ = node->get_parameter(name_ + ".decay_radius").as_double();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".core_cost_value", rclcpp::ParameterValue(254));
  int cost_int = node->get_parameter(name_ + ".core_cost_value").as_int();
  core_cost_value_ = static_cast<unsigned char>(std::clamp(cost_int, 0, 255));

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".gradient_power", rclcpp::ParameterValue(1.5));
  gradient_power_ = node->get_parameter(name_ + ".gradient_power").as_double();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".teammate_timeout", rclcpp::ParameterValue(2.0));
  teammate_timeout_ = node->get_parameter(name_ + ".teammate_timeout").as_double();

  // ── 创建订阅 ──
  teammate_sub_ = node->create_subscription<rm_referee_msgs::msg::GroundRobotPosition>(
    topic,
    rclcpp::SensorDataQoS(),
    [this](const rm_referee_msgs::msg::GroundRobotPosition::SharedPtr msg) {
      teammateCallback(msg);
    });

  RCLCPP_INFO(logger_,
    "TeammateCostLayer '%s' initialized: topic=%s, "
    "hero=%d engineer=%d std3=%d std4=%d sentry=%d, "
    "core=%.2fm decay=%.2fm cost=%d power=%.1f timeout=%.1fs",
    name_.c_str(), topic.c_str(),
    enable_hero_, enable_engineer_, enable_standard_3_, enable_standard_4_, enable_sentry_,
    core_radius_, decay_radius_, static_cast<int>(core_cost_value_),
    gradient_power_, teammate_timeout_);

  current_ = true;
  matchSize();
}

void TeammateCostLayer::teammateCallback(
  const rm_referee_msgs::msg::GroundRobotPosition::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(positions_mutex_);

  positions_.hero_x       = msg->hero_x;
  positions_.hero_y       = msg->hero_y;
  positions_.engineer_x   = msg->engineer_x;
  positions_.engineer_y   = msg->engineer_y;
  positions_.standard_3_x = msg->standard_3_x;
  positions_.standard_3_y = msg->standard_3_y;
  positions_.standard_4_x = msg->standard_4_x;
  positions_.standard_4_y = msg->standard_4_y;
  positions_.sentry_x     = msg->reserved;
  positions_.sentry_y     = msg->reserved_2;
  positions_.stamp        = node_.lock()->now();
  positions_.valid        = true;
}

bool TeammateCostLayer::isDataValid() const
{
  std::lock_guard<std::mutex> lock(positions_mutex_);
  if (!positions_.valid) {
    return false;
  }
  auto node = node_.lock();
  if (!node) {
    return false;
  }
  auto now = node->now();
  return (now - positions_.stamp).seconds() < teammate_timeout_;
}

void TeammateCostLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(positions_mutex_);
  if (!isDataValid()) {
    return;
  }

  const double total_radius = core_radius_ + decay_radius_;

  // 扩展每个启用队友的包围盒
  auto expand = [&](double tx, double ty) {
    *min_x = std::min(*min_x, tx - total_radius);
    *min_y = std::min(*min_y, ty - total_radius);
    *max_x = std::max(*max_x, tx + total_radius);
    *max_y = std::max(*max_y, ty + total_radius);
  };

  if (enable_hero_)       expand(positions_.hero_x, positions_.hero_y);
  if (enable_engineer_)   expand(positions_.engineer_x, positions_.engineer_y);
  if (enable_standard_3_) expand(positions_.standard_3_x, positions_.standard_3_y);
  if (enable_standard_4_) expand(positions_.standard_4_x, positions_.standard_4_y);
  if (enable_sentry_)     expand(positions_.sentry_x, positions_.sentry_y);
}

void TeammateCostLayer::applyTeammateCost(
  nav2_costmap_2d::Costmap2D & master_grid,
  double tx, double ty)
{
  const double total_radius = core_radius_ + decay_radius_;
  const unsigned int size_x = master_grid.getSizeInCellsX();
  const unsigned int size_y = master_grid.getSizeInCellsY();

  // ── 世界坐标 → 网格坐标（带裁剪） ──
  unsigned int cx_min = 0, cy_min = 0;
  unsigned int cx_max = size_x - 1, cy_max = size_y - 1;

  if (!master_grid.worldToMap(tx - total_radius, ty - total_radius, cx_min, cy_min)) {
    cx_min = 0;
    cy_min = 0;
  }
  if (!master_grid.worldToMap(tx + total_radius, ty + total_radius, cx_max, cy_max)) {
    cx_max = size_x - 1;
    cy_max = size_y - 1;
  }

  cx_min = std::max(0u, cx_min);
  cy_min = std::max(0u, cy_min);
  cx_max = std::min(size_x - 1, cx_max);
  cy_max = std::min(size_y - 1, cy_max);

  // ── 遍历受影响网格 ──
  for (unsigned int cy = cy_min; cy <= cy_max; ++cy) {
    for (unsigned int cx = cx_min; cx <= cx_max; ++cx) {
      double wx, wy;
      master_grid.mapToWorld(cx, cy, wx, wy);

      double dx = wx - tx;
      double dy = wy - ty;
      double dist = std::hypot(dx, dy);

      unsigned char new_cost = 0;

      if (dist <= core_radius_) {
        // 核心致命区
        new_cost = core_cost_value_;
      } else if (dist <= total_radius) {
        // 渐变衰减区: cost = peak × (1 - (d-core)/decay)^power
        double t = (dist - core_radius_) / decay_radius_;
        new_cost = static_cast<unsigned char>(
          core_cost_value_ * std::pow(1.0 - t, gradient_power_));
      } else {
        continue;
      }

      // max 策略：只在代价更高时写入
      unsigned char current = master_grid.getCost(cx, cy);
      if (new_cost > current) {
        master_grid.setCost(cx, cy, new_cost);
      }
    }
  }
}

void TeammateCostLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/)
{
  if (!enabled_) {
    return;
  }

  // 获取队友位置的本地拷贝（最小化锁持有时间）
  TeammatePositions pos_copy;
  {
    std::lock_guard<std::mutex> lock(positions_mutex_);
    if (!isDataValid()) {
      return;
    }
    pos_copy = positions_;
  }

  // 对每个启用的队友应用代价
  if (enable_hero_)       applyTeammateCost(master_grid, pos_copy.hero_x, pos_copy.hero_y);
  if (enable_engineer_)   applyTeammateCost(master_grid, pos_copy.engineer_x, pos_copy.engineer_y);
  if (enable_standard_3_) applyTeammateCost(master_grid, pos_copy.standard_3_x, pos_copy.standard_3_y);
  if (enable_standard_4_) applyTeammateCost(master_grid, pos_copy.standard_4_x, pos_copy.standard_4_y);
  if (enable_sentry_)     applyTeammateCost(master_grid, pos_copy.sentry_x, pos_copy.sentry_y);

  current_ = true;
}

void TeammateCostLayer::matchSize()
{
  // 本层没有内部数组需要调整大小
}

void TeammateCostLayer::reset()
{
  current_ = false;
}

}  // namespace teammate_cost_layer

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  teammate_cost_layer::TeammateCostLayer, nav2_costmap_2d::Layer)

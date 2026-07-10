// pri_adaptive_mppi.cpp
// 自适应MPPI控制器wrapper实现 — 支持多条直线

#include "pri_adaptive_mppi/pri_adaptive_mppi.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace pri_adaptive_mppi
{

// ═══════════════════════════════════════════════════════
//  生命周期: configure
// ═══════════════════════════════════════════════════════

void PriAdaptiveMppi::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("PriAdaptiveMppi: parent node 已失效");
  }

  node_ = parent;
  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  logger_ = node->get_logger();

  // ── 1. 声明插件专属参数 ──
  declareCommonParams();

  node->get_parameter(name + ".inner_plugin", inner_plugin_type_);
  node->get_parameter(name + ".max_path_age", max_path_age_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".uphill_braking_distance",
    rclcpp::ParameterValue(0.5));
  uphill_braking_distance_ = node->get_parameter(
    name + ".uphill_braking_distance").as_double();

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".uphill_stuck_time",
    rclcpp::ParameterValue(2.0));
  uphill_stuck_time_ = node->get_parameter(
    name + ".uphill_stuck_time").as_double();

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".uphill_stuck_distance",
    rclcpp::ParameterValue(0.1));
  uphill_stuck_distance_ = node->get_parameter(
    name + ".uphill_stuck_distance").as_double();

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".uphill_stuck_vw",
    rclcpp::ParameterValue(0.5));
  uphill_stuck_vw_ = node->get_parameter(
    name + ".uphill_stuck_vw").as_double();

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".uphill_stuck_duration",
    rclcpp::ParameterValue(1.0));
  uphill_stuck_duration_ = node->get_parameter(
    name + ".uphill_stuck_duration").as_double();

  // ── 2. 读取直线列表 ──
  std::vector<std::string> line_names;
  node->get_parameter(name + ".lines", line_names);

  RCLCPP_INFO(logger_, "PriAdaptiveMppi: 配置 %zu 条直线", line_names.size());

  for (size_t i = 0; i < line_names.size(); ++i) {
    LineConfig lc;
    lc.name = line_names[i];

    // 读取并声明直线几何参数
    std::string prefix = name + "." + lc.name;

    nav2_util::declare_parameter_if_not_declared(
      node, prefix + ".point_1",
      rclcpp::ParameterValue(std::vector<double>{0.0, 0.0}));
    std::vector<double> lp1;
    node->get_parameter(prefix + ".point_1", lp1);
    if (lp1.size() >= 2) { lc.x1 = lp1[0]; lc.y1 = lp1[1]; }

    nav2_util::declare_parameter_if_not_declared(
      node, prefix + ".point_2",
      rclcpp::ParameterValue(std::vector<double>{5.0, 0.0}));
    std::vector<double> lp2;
    node->get_parameter(prefix + ".point_2", lp2);
    if (lp2.size() >= 2) { lc.x2 = lp2[0]; lc.y2 = lp2[1]; }

    nav2_util::declare_parameter_if_not_declared(
      node, prefix + ".inflation_radius_uphill",
      rclcpp::ParameterValue(1.0));
    node->get_parameter(prefix + ".inflation_radius_uphill", lc.inflation_radius_uphill);

    nav2_util::declare_parameter_if_not_declared(
      node, prefix + ".inflation_radius_downhill",
      rclcpp::ParameterValue(1.0));
    node->get_parameter(prefix + ".inflation_radius_downhill", lc.inflation_radius_downhill);

    nav2_util::declare_parameter_if_not_declared(
      node, prefix + ".enable_braking",
      rclcpp::ParameterValue(false));
    node->get_parameter(prefix + ".enable_braking", lc.enable_braking);

    // 预计算几何
    lc.dx = lc.x2 - lc.x1;
    lc.dy = lc.y2 - lc.y1;
    lc.length = std::hypot(lc.dx, lc.dy);

    // 初始化可视化
    initLineMarkers(lc, static_cast<int>(i));

    lines_.push_back(std::move(lc));

    RCLCPP_INFO(logger_,
      "  直线[%s]: (%.2f,%.2f)→(%.2f,%.2f) 上坡膨胀=%.2f m, 下坡膨胀=%.2f m",
      lc.name.c_str(), lc.x1, lc.y1, lc.x2, lc.y2,
      lc.inflation_radius_uphill, lc.inflation_radius_downhill);
  }

  // ── 3. 加载内部 MPPI（首次使用 normal 模式） ──
  loader_ = std::make_unique<pluginlib::ClassLoader<nav2_core::Controller>>(
    "nav2_core", "nav2_core::Controller");

  inner_controller_ = loader_->createUniqueInstance(inner_plugin_type_);
  std::string normal_name = plugin_name_ + "." + PREFIX_NORMAL;
  inner_controller_->configure(parent, normal_name, tf_, costmap_ros_);

  // ── 4. 初始化可视化发布器 ──
  // Transient Local 确保 RViz 启动后能收到已发布的静态 Marker
  viz_pub_ = node->create_publisher<visualization_msgs::msg::Marker>(
    plugin_name_ + "/adaptive_line_visualization",
    rclcpp::QoS(10).transient_local().reliable());

  vw_pub_ = node->create_publisher<sentry_msgs::msg::Vw>(
    "/vw", rclcpp::QoS(10));

  RCLCPP_INFO(logger_,
    "PriAdaptiveMppi 配置完成: %zu 条直线, 内部控制器=%s",
    lines_.size(), inner_plugin_type_.c_str());
}

// ═══════════════════════════════════════════════════════
//  生命周期: cleanup / activate / deactivate
// ═══════════════════════════════════════════════════════

void PriAdaptiveMppi::cleanup()
{
  if (inner_controller_) { inner_controller_->cleanup(); }
  viz_pub_.reset();
}

void PriAdaptiveMppi::activate()
{
  if (inner_controller_) { inner_controller_->activate(); }
}

void PriAdaptiveMppi::deactivate()
{
  if (inner_controller_) { inner_controller_->deactivate(); }
}

// ═══════════════════════════════════════════════════════
//  setPlan — 缓存路径 + 委托
// ═══════════════════════════════════════════════════════

void PriAdaptiveMppi::setPlan(const nav_msgs::msg::Path & path)
{
  current_path_ = path;
  last_path_time_ = rclcpp::Clock(RCL_ROS_TIME).now();
  path_received_ = true;

  if (inner_controller_) {
    inner_controller_->setPlan(path);
  }
}

// ═══════════════════════════════════════════════════════
//  setSpeedLimit — 委托内部 MPPI
// ═══════════════════════════════════════════════════════

void PriAdaptiveMppi::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (inner_controller_) {
    inner_controller_->setSpeedLimit(speed_limit, percentage);
  }
}

// ═══════════════════════════════════════════════════════
//  computeVelocityCommands — 核心决策循环
// ═══════════════════════════════════════════════════════

geometry_msgs::msg::TwistStamped PriAdaptiveMppi::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  // ── 1. 遍历所有直线，判断当前应使用的模式 ──
  int best_line = -1;
  CrossingMode best_mode = CrossingMode::NORMAL;
  double best_dist = std::numeric_limits<double>::max();

  if (path_received_) {
    auto now = rclcpp::Clock(RCL_ROS_TIME).now();
    double path_age = (now - last_path_time_).seconds();

    if (path_age <= max_path_age_) {
      for (size_t i = 0; i < lines_.size(); ++i) {
        const auto & line = lines_[i];

        // 使用有符号距离判断机器人处于上坡侧还是下坡侧
        double d_signed = signedDistanceToLine(
          pose.pose.position.x, pose.pose.position.y,
          line.x1, line.y1, line.x2, line.y2);
        // 用线段距离（非无限直线）限定膨胀区范围
        double d_perp = distanceToSegment(
          pose.pose.position.x, pose.pose.position.y,
          line.x1, line.y1, line.x2, line.y2, line.length);

        // 根据所在侧选择对应的膨胀半径
        double inflation = (d_signed > 0.0)
          ? line.inflation_radius_uphill
          : line.inflation_radius_downhill;

        if (d_perp <= inflation && d_perp < best_dist) {
          best_line = static_cast<int>(i);
          best_dist = d_perp;
        }
      }
    }
  }

  // ── 2. 确定穿越模式（滞留 + 方向反转检测）──
  if (best_line >= 0) {
    const auto & line = lines_[best_line];
    int crossing = detectPathCrossing(
      current_path_, line.x1, line.y1, line.x2, line.y2);

    // 若 best_line 未检测到穿越，轮询所有边（可能路径穿的是另一条边）
    if (crossing == 0 && active_crossing_ != CrossingMode::NORMAL) {
      for (size_t i = 0; i < lines_.size(); ++i) {
        if (static_cast<int>(i) == best_line) continue;
        crossing = detectPathCrossing(
          current_path_, lines_[i].x1, lines_[i].y1,
          lines_[i].x2, lines_[i].y2);
        if (crossing != 0) break;
      }
    }

    if (active_crossing_ == CrossingMode::NORMAL) {
      // 首次进入膨胀区 —— 检测穿越方向
      if (crossing > 0) {
        best_mode = CrossingMode::UPHILL;
      } else if (crossing < 0) {
        best_mode = CrossingMode::DOWNHILL;
      }
      // crossing == 0: 在膨胀区内但路径未穿越 → 保持 normal
    } else {
      // 已在穿越模式中 —— 保持，除非检测到方向反转
      if (crossing != 0) {
        CrossingMode new_dir = (crossing > 0)
          ? CrossingMode::UPHILL : CrossingMode::DOWNHILL;
        if (new_dir != active_crossing_) {
          best_mode = new_dir;  // 方向反转：允许切换
        } else {
          best_mode = active_crossing_;  // 同方向：保持
        }
      } else {
        best_mode = active_crossing_;  // 未穿越：保持滞留
      }
    }
  }
  // best_line < 0: 不在任何膨胀区内 → best_mode 已是 NORMAL

  // ── 3. 模式改变时切换 ──
  if (best_line != active_line_index_ || best_mode != active_crossing_) {
    switchMode(best_line, best_mode);
  }

  // ── 4. 膨胀区内卡住检测 + Vw 输出（与上下坡模式无关）──
  auto now_stuck = rclcpp::Clock(RCL_ROS_TIME).now();
  static int prev_best_line = -1;

  if (best_line >= 0) {
    // 刚进入膨胀区（含离开后重进）→ 重置计时
    if (prev_best_line < 0) {
      uphill_start_time_ = now_stuck;
      uphill_start_pose_ = pose.pose;
      stuck_override_ = false;
    }

    // 仅当车辆有非零速度（在尝试移动）时才检测卡住
    bool is_moving = (std::abs(velocity.linear.x) > 1e-3 ||
                      std::abs(velocity.linear.y) > 1e-3);

    if (is_moving && !stuck_override_) {
      double elapsed = (now_stuck - uphill_start_time_).seconds();
      double dx = pose.pose.position.x - uphill_start_pose_.position.x;
      double dy = pose.pose.position.y - uphill_start_pose_.position.y;
      double moved = std::hypot(dx, dy);

      if (elapsed >= uphill_stuck_time_ && moved < uphill_stuck_distance_) {
        stuck_override_ = true;
        stuck_override_end_ = now_stuck +
          rclcpp::Duration::from_seconds(uphill_stuck_duration_);
        RCLCPP_WARN(logger_,
          "Stuck in zone: %.1fs 移动%.3fm < %.3fm, Vw=%.2f x%.1fs",
          elapsed, moved, uphill_stuck_distance_,
          uphill_stuck_vw_, uphill_stuck_duration_);
      }
    }

    if (stuck_override_) {
      if (now_stuck < stuck_override_end_) {
        sentry_msgs::msg::Vw vw_msg;
        vw_msg.vw = static_cast<float>(uphill_stuck_vw_);
        vw_pub_->publish(vw_msg);
      } else {
        stuck_override_ = false;
        // Vw 结束 → 重置，本轮可再次触发
        uphill_start_time_ = now_stuck;
        uphill_start_pose_ = pose.pose;
      }
    }
  }
  prev_best_line = best_line;

  // ── 5. 发布可视化 ──
  publishVisualization();

  // ── 6. 委托给内部 MPPI ──
  if (!inner_controller_) {
    geometry_msgs::msg::TwistStamped empty_cmd;
    empty_cmd.header = pose.header;
    return empty_cmd;
  }

  auto cmd_vel = inner_controller_->computeVelocityCommands(pose, velocity, goal_checker);

  // ── 7. 上坡急停：越线后在下坡侧距离比例刹车 ──
  if (active_crossing_ == CrossingMode::UPHILL &&
      active_line_index_ >= 0 &&
      static_cast<size_t>(active_line_index_) < lines_.size()) {
    const auto & line = lines_[active_line_index_];
    if (line.enable_braking && uphill_braking_distance_ > 0.0) {
      double d_signed = signedDistanceToLine(
        pose.pose.position.x, pose.pose.position.y,
        line.x1, line.y1, line.x2, line.y2);
      // 仅在下坡侧（内侧, d_signed < 0）刹车
      if (d_signed < 0.0) {
        double d_perp = std::abs(d_signed) / line.length;
        double scale = std::clamp(d_perp / uphill_braking_distance_, 0.0, 1.0);
        cmd_vel.twist.linear.x  *= scale;
        cmd_vel.twist.linear.y  *= scale;
        cmd_vel.twist.angular.z *= scale;
      }
    }
  }

  return cmd_vel;
}

// ═══════════════════════════════════════════════════════
//  模式切换
// ═══════════════════════════════════════════════════════

const char * PriAdaptiveMppi::modeToPrefix(CrossingMode mode)
{
  switch (mode) {
    case CrossingMode::NORMAL:   return PREFIX_NORMAL;
    case CrossingMode::UPHILL:   return PREFIX_UPHILL;
    case CrossingMode::DOWNHILL: return PREFIX_DOWNHILL;
  }
  return PREFIX_NORMAL;
}

void PriAdaptiveMppi::switchMode(int line_index, CrossingMode new_mode)
{
  const char * new_prefix = modeToPrefix(new_mode);

  auto node = node_.lock();
  if (!node || !inner_controller_) {
    RCLCPP_ERROR(logger_, "switchMode: 节点或内部控制器无效");
    return;
  }

  // ── 构建参数命名空间（上下坡参数全局共享，不按直线区分）──
  std::string prefixed_name;
  if (line_index >= 0 && static_cast<size_t>(line_index) < lines_.size()) {
    const auto & line = lines_[line_index];
    prefixed_name = plugin_name_ + "." + new_prefix;

    // 中文模式名
    const char * mode_cn = "普通";
    if (new_mode == CrossingMode::UPHILL)   mode_cn = "上坡";
    if (new_mode == CrossingMode::DOWNHILL) mode_cn = "下坡";

    RCLCPP_INFO(logger_, "🔄 使用 MPPI 参数集: 【%s】 (直线=%s, 前缀=%s)",
                mode_cn, line.name.c_str(), new_prefix);
  } else {
    prefixed_name = plugin_name_ + "." + PREFIX_NORMAL;
    RCLCPP_INFO(logger_, "🔄 使用 MPPI 参数集: 【普通】 (正常导航)");
  }

  // ── 1. 清理旧实例 ──
  inner_controller_->deactivate();
  inner_controller_->cleanup();
  inner_controller_.reset();

  // ── 2. 创建新实例，用新模式前缀配置 ──
  inner_controller_ = loader_->createUniqueInstance(inner_plugin_type_);
  inner_controller_->configure(node_, prefixed_name, tf_, costmap_ros_);
  inner_controller_->activate();

  // ── 3. 重新设置路径 ──
  if (path_received_) {
    inner_controller_->setPlan(current_path_);
  }

  // ── 4. 更新状态 ──
  active_line_index_ = line_index;
  active_crossing_ = new_mode;
}

// ═══════════════════════════════════════════════════════
//  参数声明
// ═══════════════════════════════════════════════════════

void PriAdaptiveMppi::declareCommonParams()
{
  auto node = node_.lock();
  if (!node) { return; }

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".inner_plugin",
    rclcpp::ParameterValue("nav2_mppi_controller::MPPIController"));

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_path_age",
    rclcpp::ParameterValue(2.0));

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".uphill_braking_distance",
    rclcpp::ParameterValue(0.5));

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".uphill_stuck_time",
    rclcpp::ParameterValue(2.0));

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".uphill_stuck_distance",
    rclcpp::ParameterValue(0.1));

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".uphill_stuck_vw",
    rclcpp::ParameterValue(0.5));

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".uphill_stuck_duration",
    rclcpp::ParameterValue(1.0));

  // lines 列表 — 每条直线的名称
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".lines",
    rclcpp::ParameterValue(std::vector<std::string>{}));
}

// ═══════════════════════════════════════════════════════
//  可视化
// ═══════════════════════════════════════════════════════

void PriAdaptiveMppi::initLineMarkers(LineConfig & line, int ns_suffix)
{
  // ns_suffix 用于在多个直线间区分 marker namespace

  // ── 直线 Marker (LINE_STRIP) ──
  line.line_marker.header.frame_id = "map";
  line.line_marker.ns = "pri_adaptive/line_" + std::to_string(ns_suffix);
  line.line_marker.id = 0;
  line.line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line.line_marker.action = visualization_msgs::msg::Marker::ADD;
  line.line_marker.scale.x = 0.05;
  line.line_marker.color.a = 1.0;
  line.line_marker.color.r = 1.0;
  line.line_marker.color.g = 1.0;
  line.line_marker.color.b = 1.0;
  line.line_marker.pose.orientation.w = 1.0;

  geometry_msgs::msg::Point lp1, lp2;
  lp1.x = line.x1; lp1.y = line.y1; lp1.z = 0.0;
  lp2.x = line.x2; lp2.y = line.y2; lp2.z = 0.0;
  line.line_marker.points = {lp1, lp2};

  // ── 上坡侧膨胀区 Marker (LINE_LIST) ──
  line.uphill_zone_marker.header.frame_id = "map";
  line.uphill_zone_marker.ns = "pri_adaptive/uphill_" + std::to_string(ns_suffix);
  line.uphill_zone_marker.id = 1;
  line.uphill_zone_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  line.uphill_zone_marker.action = visualization_msgs::msg::Marker::ADD;
  line.uphill_zone_marker.scale.x = 0.02;
  line.uphill_zone_marker.color.a = 0.3;
  line.uphill_zone_marker.color.r = 1.0;
  line.uphill_zone_marker.color.g = 1.0;
  line.uphill_zone_marker.color.b = 0.0;
  line.uphill_zone_marker.pose.orientation.w = 1.0;
  line.uphill_zone_marker.points = line.buildZoneLines(+1.0, line.inflation_radius_uphill);

  // ── 下坡侧膨胀区 Marker (LINE_LIST) ──
  line.downhill_zone_marker.header.frame_id = "map";
  line.downhill_zone_marker.ns = "pri_adaptive/downhill_" + std::to_string(ns_suffix);
  line.downhill_zone_marker.id = 2;
  line.downhill_zone_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  line.downhill_zone_marker.action = visualization_msgs::msg::Marker::ADD;
  line.downhill_zone_marker.scale.x = 0.02;
  line.downhill_zone_marker.color.a = 0.3;
  line.downhill_zone_marker.color.r = 1.0;
  line.downhill_zone_marker.color.g = 1.0;
  line.downhill_zone_marker.color.b = 0.0;
  line.downhill_zone_marker.pose.orientation.w = 1.0;
  line.downhill_zone_marker.points = line.buildZoneLines(-1.0, line.inflation_radius_downhill);
}

void PriAdaptiveMppi::publishVisualization()
{
  if (!viz_pub_) { return; }

  auto node = node_.lock();
  if (!node) { return; }

  auto now = node->now();

  for (size_t i = 0; i < lines_.size(); ++i) {
    auto & line = lines_[i];

    line.line_marker.header.stamp = now;
    line.uphill_zone_marker.header.stamp = now;
    line.downhill_zone_marker.header.stamp = now;

    // 根据是否为活跃直线 + 穿越模式修改颜色
    bool is_active = (active_line_index_ == static_cast<int>(i));
    if (is_active) {
      switch (active_crossing_) {
        case CrossingMode::UPHILL:
          // 红色 — 上坡穿越
          line.line_marker.color.r = 1.0;
          line.line_marker.color.g = 0.3;
          line.line_marker.color.b = 0.3;
          line.line_marker.scale.x = 0.08;  // 加粗
          break;
        case CrossingMode::DOWNHILL:
          // 蓝色 — 下坡穿越
          line.line_marker.color.r = 0.3;
          line.line_marker.color.g = 0.3;
          line.line_marker.color.b = 1.0;
          line.line_marker.scale.x = 0.08;
          break;
        default:
          break;
      }
    } else {
      // 非活跃直线 — 灰色细线
      line.line_marker.color.r = 0.5;
      line.line_marker.color.g = 0.5;
      line.line_marker.color.b = 0.5;
      line.line_marker.scale.x = 0.03;
    }

    viz_pub_->publish(line.line_marker);
    viz_pub_->publish(line.uphill_zone_marker);
    viz_pub_->publish(line.downhill_zone_marker);
  }
}

}  // namespace pri_adaptive_mppi

PLUGINLIB_EXPORT_CLASS(
  pri_adaptive_mppi::PriAdaptiveMppi,
  nav2_core::Controller)

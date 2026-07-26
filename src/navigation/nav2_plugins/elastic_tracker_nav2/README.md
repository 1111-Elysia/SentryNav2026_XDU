# elastic_tracker_nav2

Nav2 GlobalPlanner 插件，实现基于 MINCO (MINimum COntrol) 多项式轨迹优化的动态目标追踪规划器。改编自 [Elastic-Tracker](https://github.com/jialinji/Elastic-Tracker) (Jialin Ji et al., ICRA 2022)，集成到 ROS2 Nav2 框架中。

## 核心思想

将追踪问题建模为**弹簧阻尼系统**：在目标后方 `tracking_dist` 处设定虚拟追踪点（弹簧平衡位置），通过 A* 全局搜索 + MINCO 轨迹优化生成平滑、安全、满足运动学约束的追踪路径。

## 规划流水线

```
┌─────────────┐
│ 目标位置订阅  │  /detected_target_pose (PoseStamped)
│ + EKF 速度估计│
└──────┬──────┘
       │
┌──────▼──────┐
│ 1. 目标预测  │  Predict2D: 加速度空间 A* 搜索
│              │  输出: pre_dur 秒内的预测轨迹
└──────┬──────┘
       │
┌──────▼──────┐
│ 2. 弹簧目标  │  track_goal = target + dir/|dir| × tracking_dist
│              │  死区: 距离<0.15m 且目标速度<0.2m/s → 悬停
└──────┬──────┘
       │
┌──────▼──────┐
│ 3. A* 路径   │  Env2D: costmap 代价感知的 8-邻域 A*
│              │  50ms 超时保护，直线回退
└──────┬──────┘
       │
┌──────▼──────┐
│ 4. 路径简化  │  Ramer-Douglas-Peucker 式贪心最远点选取
│              │  上限 ~12 个路径点，控制 MINCO 变量数
└──────┬──────┘
       │
┌──────▼──────┐
│ 5. 安全走廊  │  CorridorGenerator: 沿路径生成凸多边形 SFC
│              │  corridor_width 控制走廊宽度
└──────┬──────┘
       │
┌──────▼──────┐
│ 6. MINCO优化 │  5 阶多项式轨迹 + 走廊约束 + L-BFGS 求解
│              │  输出: dt=0.05s 采样的平滑路径
└──────┬──────┘
       │
┌──────▼──────┐
│ 7. 路径平滑  │  SmacPlanner2D Smoother (回退路径专用)
└─────────────┘
```

## 两种工作模式

### 追踪模式 (`use_tracking: true`)

- 订阅 `/detected_target_pose` 获取实时目标位置
- EKF 从位置观测估计目标速度（α-β 滤波器）
- 内部 10Hz 定时器独立重规划（不依赖 Nav2 BT 触发）
- `createPlan()` 直接返回缓存的最新路径
- 目标丢失超过 `target_timeout` → 悬停
- 目标离开 `tracking_zone` 多边形区域 → 不发路径

### 静态模式 (`use_tracking: false`)

- 退化为标准 Nav2 GlobalPlanner
- `createPlan(start, goal)` 执行一次性 A* + 平滑
- 适用于普通导航任务

## 插件信息

| 属性 | 值 |
|------|-----|
| 插件类 | `elastic_planner::ElasticPlanner` |
| 基类 | `nav2_core::GlobalPlanner` |
| 插件名（yaml 配置） | `elastic_planner::ElasticPlanner` |

## 参数

### 追踪参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `tracking_dist` | float | 1.0 | 期望跟踪距离 (m)，弹簧平衡位置 |
| `tracking_dur` | float | 3.0 | 目标预测时域 (s) |
| `tracking_dt` | float | 0.2 | 预测时间步长 (s) |
| `use_tracking` | bool | true | 启用追踪模式 |
| `target_timeout` | float | 1.0 | 目标丢失超时 (s)，超时后悬停 |
| `target_topic` | string | `/detected_target_pose` | 目标位姿订阅话题 |

### EKF 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `ekf_enabled` | bool | true | 启用 EKF 目标速度估计 |
| `ekf_alpha` | float | 0.1 | 位置平滑增益（越小越平滑） |
| `ekf_beta` | float | 0.05 | 速度平滑增益 |
| `ekf_reset_dt` | float | 3.0 | EKF 重置时间阈值 (s) |

### A* 搜索参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `cost_weight` | float | 1.5 | costmap 代价权重，`1 + w*cost/254` |
| `minimum_turning_radius` | float | 0.05 | 最小转弯半径 (m)（传给 Smoother） |
| `max_planning_time` | float | 4.5 | Smoother 最大优化时间 (s) |

### MINCO 优化参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `minco_enabled` | bool | true | 启用 MINCO 轨迹优化 |
| `minco_sample_dt` | float | 0.05 | 输出路径采样步长 (s) |
| `minco_nominal_speed` | float | 1.0 | 名义巡航速度 (m/s) |
| `minco_min_piece_duration` | float | 0.15 | 最小轨迹段时长 (s) |
| `corridor_width` | float | 0.8 | 安全走廊宽度 (m) |

### L-BFGS 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `lbfgs_enabled` | bool | true | 启用 L-BFGS 数值优化 |
| `lbfgs_max_iterations` | int | 50 | 最大迭代次数 |
| `lbfgs_g_epsilon` | float | 1e-4 | 梯度收敛阈值 |
| `finite_diff_eps` | float | 1e-4 | 有限差分步长 |
| `corridor_weight` | float | 50.0 | 走廊约束惩罚权重 |

### 追踪区域

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `tracking_zone_vertices` | float[] | `[]` | 追踪有效区域多边形顶点 `[x1,y1,x2,y2,...]`，空 = 不限 |

## 在 Nav2 中使用

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased", "ElasticTracker"]

    GridBased:
      plugin: "nav2_smac_planner/SmacPlanner2D"
      # ... 标准 SmacPlanner 参数 ...

    ElasticTracker:
      plugin: "elastic_planner::ElasticPlanner"   # 本插件
      tracking_dist: 1.0
      tracking_dur: 3.0
      tracking_dt: 0.2
      use_tracking: true
      ekf_enabled: true
      target_timeout: 1.0
      target_topic: "/detected_target_pose"
      cost_weight: 5.0
      minco_enabled: true
      corridor_width: 0.8
      minimum_turning_radius: 0.05
      max_planning_time: 4.5
      # 追踪区域约束 (可选)
      tracking_zone_vertices: [0.0, 0.0, 5.0, 0.0, 7.0, 5.0, 3.0, 5.0, 0.0, 3.0]
      # Smoother 参数 (SmacPlanner2D)
      smoother.tolerance: 1.0e-10
      smoother.max_iterations: 10000
      smoother.w_data: 0.1
      smoother.w_smooth: 0.4
      smoother.do_refinement: true
```

> [!IMPORTANT]
> 追踪模式下，行为树需要包含 `PlannerSelector` 和 `GoalUpdater` 节点，以支持规划器切换和目标动态更新。参考 `sim` 包中的 `tracking_bt.xml`。

## 话题

| 话题 | 方向 | 类型 | 说明 |
|------|------|------|------|
| `/detected_target_pose`（可配置） | 订阅 | `geometry_msgs/PoseStamped` | 目标实时位姿 |
| `/tracking_zone` | 发布 | `geometry_msgs/PolygonStamped` | 追踪有效区域多边形（1Hz，RViz 可视化） |

## 模块概览

| 模块 | 命名空间 | 功能 |
|------|----------|------|
| `ElasticPlanner` | `elastic_planner` | Nav2 GlobalPlanner 主类，生命周期 + 定时重规划 |
| `Env2D` | `env_2d` | costmap 代价感知的 8-邻域 A* 搜索（50ms 超时） |
| `Predict2D` | `prediction_2d` | 加速度空间 A* 搜索预测目标轨迹（50ms 超时） |
| `CorridorGenerator` | `elastic_tracker` | 沿 A* 路径生成凸多边形安全飞行走廊 (SFC) |
| `MincoOptimizer` | `elastic_tracker` | MINCO 5 阶多项式轨迹优化 + 走廊约束 + L-BFGS |
| `PolygonZone` | `elastic_planner` | 射线投射多边形区域判定 + RViz 发布 |

## 编译

```bash
colcon build --packages-select elastic_tracker_nav2
```

依赖：
- `rclcpp` / `rclcpp_lifecycle` / `pluginlib`
- `nav2_core` / `nav2_costmap_2d` / `nav2_smac_planner`
- `tf2_ros` / `tf2_geometry_msgs`
- `Eigen3`
- 内置的 GCOPTER MINCO / L-BFGS 头文件（MIT 许可证），无需依赖外部 `gcopter` 包或源码目录

## 文件结构

```
elastic_tracker_nav2/
├── CMakeLists.txt
├── package.xml
├── plugins.xml                          # pluginlib 导出描述
├── include/
│   ├── elastic_planner/
│   │   └── elastic_planner.hpp          # 主规划器 (Nav2 GlobalPlanner)
│   ├── elastic_tracker/
│   │   ├── corridor_2d.hpp              # SFC 安全走廊生成
│   │   ├── minco_optimizer.hpp          # MINCO 轨迹优化 + L-BFGS
│   │   └── 3rdparty/gcopter/             # 内置 MINCO / L-BFGS 头文件（MIT）
│   ├── env/
│   │   ├── env_2d.hpp                   # costmap A* 搜索环境
│   │   └── polygon_zone.hpp             # 多边形区域判定
│   └── prediction/
│       └── prediction_2d.hpp            # 加速度空间目标预测
└── src/
    ├── elastic_planner.cpp              # 主规划器实现
    ├── corridor_2d.cpp                  # 走廊生成实现
    └── minco_optimizer.cpp              # MINCO 优化实现
```

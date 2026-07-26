# goal_approach_controller

Nav2 Controller 装饰器（Wrapper），透明代理内部控制器（如 MPPI），在接近目标点时自动限制速度，防止高速冲过目标。采用 pluginlib 动态加载内部控制器，对外完全兼容 Nav2 Controller 接口。

## 工作原理

```
到目标距离 dist
      │
      ▼
┌──────────────────────────────────────────────────────┐
│  dist > approach_distance                            │
│  → 透明透传，不做任何干预                               │
├──────────────────────────────────────────────────────┤
│  direct_approach_distance < dist < approach_distance │
│  → 速度钳位：合速度 > approach_velocity 时等比缩放      │
│    角速度同步缩放，避免原地打转                          │
├──────────────────────────────────────────────────────┤
│  dist < direct_approach_distance                     │
│  → 直接驱动模式：绕过内部控制器，直接朝目标点走            │
│    线速度 = min(approach_velocity, dist × kp)         │
│    方向 = (dx, dy) / dist                             │
│    角速度 = 0                                         │
└──────────────────────────────────────────────────────┘
```

两种钳位策略配合：**速度钳位**（不改变方向，仅减速）平滑过渡到**直接驱动**（绕过控制器弧线输出，直线逼近目标）。

## 插件信息

| 属性 | 值 |
|------|-----|
| 插件类 | `goal_approach_controller::GoalApproachController` |
| 基类 | `nav2_core::Controller` |
| 插件名（yaml 配置） | `goal_approach_controller::GoalApproachController` |

## 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `inner_plugin` | string | `nav2_mppi_controller::MPPIController` | 内部控制器插件名 |
| `approach_distance` | float | 1.5 | 进入速度钳位区的距离阈值 (m) |
| `approach_velocity` | float | 0.5 | 钳位区的最大合速度 (m/s) |
| `direct_approach_distance` | float | 0.5 | 进入直接驱动区的距离阈值 (m) |
| `direct_approach_kp` | float | 1.0 | 直接驱动区的 P 增益，`v = min(v_max, dist × kp)` |

## 在 Nav2 中使用

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]

    FollowPath:
      plugin: "goal_approach_controller::GoalApproachController"  # 本插件
      inner_plugin: "nav2_mppi_controller::MPPIController"        # 内部控制器
      approach_distance: 1.5           # 1.5m 外全速
      approach_velocity: 0.5           # 1.5m 内限速 0.5 m/s
      direct_approach_distance: 0.5    # 0.5m 内直线逼近
      direct_approach_kp: 1.0          # P 增益

      # 以下参数透传给内部 MPPI 控制器
      time_steps: 60
      model_dt: 0.05
      # ... 其余 MPPI 参数 ...
```

> [!NOTE]
> 本插件通过 pluginlib 加载内部控制器，内部控制器的参数与此 wrapper 共享同一个命名空间（即 `FollowPath.*`）。内部控制器需要的所有参数（如 MPPI 的 `time_steps`, `critics` 等）写在同一层级下即可透传。

## 编译

```bash
colcon build --packages-select goal_approach_controller
```

依赖：
- `rclcpp` / `rclcpp_lifecycle` / `pluginlib`
- `nav2_core` / `nav2_util` / `nav2_costmap_2d`
- `tf2_ros` / `geometry_msgs` / `nav_msgs`

## 文件结构

```
goal_approach_controller/
├── CMakeLists.txt
├── package.xml
├── goal_approach_controller_plugin.xml   # pluginlib 导出描述
└── src/
    └── goal_approach_controller.cpp      # 实现（单文件）
```

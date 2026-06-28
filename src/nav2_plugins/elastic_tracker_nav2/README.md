# Elastic-Tracker NAV2
ROS2 Nav2 GlobalPlanner 插件，为全向轮地面机器人提供**弹簧式目标追踪**能力。

## 原理

### 弹簧追踪

插件在目标后方 `tracking_dist` 距离处维护一个虚拟追踪点（弹簧锚点），A* 路径规划始终以该锚点为目标：

```
        tracking_dist
    ○ ← ─ ─ ─ ─ ●  ─ ─ ─ ─ ─ ─ →  ○
  机器人        锚点                目标
```

- 目标靠近 → 锚点移到机器人后方 → 路径指向后方 → 自主后退拉远
- 目标远离 → 锚点移到机器人前方 → 机器人跟随
- 平衡态（距离 = tracking_dist）→ 锚点与机器人重合 → 原地保持

### 目标速度估计

插件订阅 `/detected_target_pose` 话题（只需位置，无需速度）。内部 α-β 滤波器从连续位置观测自动估计目标速度，用于运动预测。

### 规划管线

```
/detected_target_pose → EKF (α-β 滤波器)
                             ↓
                    内部 10Hz 定时器
                     ├── TF 获取机器人位姿
                     ├── 目标运动预测 (加速度空间 A*)
                     ├── 弹簧锚点计算
                     └── A* 路径搜索 (50ms 硬超时)
                             ↓
                    缓存 nav_msgs::Path
                             ↓
          createPlan() → 直接返回缓存路径
```

## 参数

| 参数 | 类型 | 默认 | 说明 |
|------|------|------|------|
| `tracking_dist` | double | 1.0 | 弹簧平衡距离 (m) |
| `tracking_dur` | double | 3.0 | 预测时域 (s) |
| `tracking_dt` | double | 0.2 | 预测时间步长 (s) |
| `use_tracking` | bool | true | 启用追踪模式 |
| `ekf_enabled` | bool | true | 启用 EKF 速度估计 |
| `target_timeout` | double | 1.0 | 目标丢失超时 (s) |
| `target_topic` | string | `/detected_target_pose` | 目标位姿话题 |

## 依赖

- ROS2 Humble
- Nav2 (nav2_core, nav2_costmap_2d, nav2_util)
- Eigen3
- pluginlib

## 使用

### 作为 Nav2 插件加载

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["ElasticTracker"]
    ElasticTracker:
      plugin: "elastic_planner::ElasticPlanner"

      # ── 跟踪参数 ───────────────────────────────────
      tracking_dist: 1.0        # 期望跟踪距离 (m)，弹簧平衡位置
      tracking_dur: 3.0         # 预测/规划时域 (s)
      tracking_dt: 0.2          # 预测时间步长 (s)

      # ── 功能开关 ───────────────────────────────────
      use_tracking: true        # 启用目标追踪模式（内部10Hz重规划定时器）
      ekf_enabled: true         # 启用EKF从位置观测估计目标速度
      target_timeout: 1.0       # 目标丢失超时 (s)，超时后原地悬停

      # ── 目标订阅 ───────────────────────────────────
      target_topic: "/detected_target_pose"  # 目标位置话题 (geometry_msgs/PoseStamped)
```

### 启动追踪

1. Nav2 就绪后，用 RViz `2D Goal Pose` 或 `NavigateToPose` 启动首次导航
2. 持续发布目标位置：

```bash
ros2 topic pub /detected_target_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 2.0, y: 2.0}}}" -r 10
```

### BT 配置

推荐在 Behavior Tree 中使用 `PlannerSelector` + `GoalUpdater`：

```xml
<PlannerSelector default_planner="ElasticTracker" topic_name="planner_name"/>
<GoalUpdater input_goal="{goal}" output_goal="{updated_goal}">
  <ComputePathToPose goal="{updated_goal}" path="{path}" planner_id="{selected_planner}"/>
</GoalUpdater>
```

`GoalUpdater` 检测 `/target_goal` 更新时自动触发重规划，无需手动干预。

## 文件结构

```
src/elastic_tracker_nav2/
├── README.md
├── CMakeLists.txt
├── package.xml
├── plugins.xml
├── include/
│   ├── elastic_planner/
│   │   └── elastic_planner.hpp      # Nav2 插件接口
│   ├── env/
│   │   └── env_2d.hpp               # A* 路径搜索 + Costmap2D 适配
│   └── prediction/
│       └── prediction_2d.hpp         # 目标运动预测
└── src/
    └── elastic_planner.cpp           # 插件实现
```


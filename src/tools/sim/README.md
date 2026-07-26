# sim

Elastic-Tracker Nav2 的 Gazebo 仿真测试环境。提供全向麦克纳姆轮追捕机器人（蓝色）和被追踪目标机器人（红色），支持一键启动 Gazebo + Nav2 + RViz，用于验证弹性追踪规划器（ElasticTracker）和自适应 MPPI 控制器（PriAdaptiveMppi）的算法效果。

该包为纯 launch/config/urdf 包（无 C++ 源码编译），依赖 `elastic_tracker_nav2`、`nav2_bringup`、`gazebo_ros` 等。

## 快速开始

### 模式一：完整弹性追踪仿真

```bash
ros2 launch sim sim.launch.py
```

启动后：
- Gazebo 中出现蓝色追捕机器人 (pursuer) 和红色目标机器人 (target)
- RViz 自动加载，显示地图、激光雷达、costmap、路径等
- 在 teleop 终端中用键盘控制红色目标移动
- 蓝色追捕机器人自动追踪目标

### 模式二：自适应 MPPI 控制器测试（无追踪）

```bash
ros2 launch sim test_adaptive_mppi.launch.py
```

启动后：
- 仅单个蓝色机器人
- 在 RViz 中使用 **"2D Goal Pose"** 工具设置导航目标点
- 机器人使用 `pri_adaptive_mppi` 控制器自主导航
- 观察 `/FollowPath/adaptive_line_visualization` 话题中的自适应直线膨胀层

## 仿真架构

```
┌──────────────────────────────────────────────────┐
│  Gazebo (empty.world)                             │
│                                                   │
│   ┌──────────┐          ┌──────────┐              │
│   │ Pursuer  │          │  Target  │              │
│   │ (Blue)   │          │  (Red)   │              │
│   │ 全向底盘  │ 追踪 ──→ │ 全向底盘  │              │
│   │ + LiDAR  │          │ (被遥控)  │              │
│   └────┬─────┘          └────┬─────┘              │
│        │ /scan              │ /target/odom         │
│        │ /odom               │                      │
│   ─────┼────────────────────┼──────────────────────│
│        ▼                     ▼                      │
│  ┌──────────────────────────────────────────┐      │
│  │             Nav2 Stack                    │      │
│  │                                           │      │
│  │  target_relay  ──→ /detected_target_pose  │      │
│  │       │                                   │      │
│  │       ▼                                   │      │
│  │  ElasticTracker Planner                   │      │
│  │       │                                   │      │
│  │       ▼                                   │      │
│  │  PriAdaptiveMppi Controller               │      │
│  │       │                                   │      │
│  │       ▼                                   │      │
│  │  /cmd_vel  ──→ pursuer                    │      │
│  └──────────────────────────────────────────┘      │
│                                                   │
│  keyboard_teleop ──→ /target/cmd_vel ──→ target   │
└──────────────────────────────────────────────────┘
```

## 机器人模型

### Pursuer（蓝色追捕者）— `urdf/omni_robot.urdf.xacro`

| 属性 | 值 |
|------|-----|
| 底盘类型 | 四麦克纳姆轮全向移动 |
| 底盘尺寸 | 0.3 × 0.25 × 0.12 m |
| 质量 | 3.0 kg |
| LiDAR | 360° 激光雷达，12m 测距，20Hz |
| 运动插件 | `libgazebo_ros_planar_move` |
| 最大速度 (MPPI) | vx/vy ±2.0 m/s, wz ±1.5 rad/s |

### Target（红色目标）— `urdf/target.urdf.xacro`

| 属性 | 值 |
|------|-----|
| 底盘类型 | 全向移动（无 LiDAR） |
| 底盘尺寸 | 0.3 × 0.25 × 0.12 m |
| 运动插件 | `libgazebo_ros_planar_move` |
| 命名空间 | `/target` |

## 话题与数据流

### Gazebo 发布

| 话题 | 发布者 | 说明 |
|------|--------|------|
| `/odom` | pursuer | 追捕者里程计 |
| `/scan` | pursuer | 360° 激光雷达扫描 |
| `/target/odom` | target | 目标里程计 |

### target_relay 转发

| 话题 | 方向 | 说明 |
|------|------|------|
| `/detected_target_pose` | 发布 | 目标实时位姿，供 ElasticTracker 订阅 |
| `/target_goal` | 发布 | 目标位姿，作为 Nav2 BT 的 `goal_updater` 输入 |
| `navigate_to_pose` (action) | 客户端 | Nav2 导航目标（首次立即发送，后续移动 >0.3m 或 10s 心跳重发） |

### Nav2 内部

| 话题 | 说明 |
|------|------|
| `/plan` | 全局路径 (ElasticTracker / SmacPlanner2D) |
| `/local_plan` | 局部路径 (MPPI) |
| `/global_costmap/costmap` | 全局代价地图 |
| `/local_costmap/costmap` | 局部代价地图 |
| `/tracking_zone` | 追踪区域多边形可视化 |
| `/FollowPath/adaptive_line_visualization` | 自适应直线膨胀层可视化 (PriAdaptiveMppi) |

## 两种 Nav2 参数配置

### 弹性追踪配置 — `config/sim_nav2_params.yaml`

| 组件 | 插件 | 说明 |
|------|------|------|
| Planner | `elastic_planner::ElasticPlanner` | 弹性追踪规划器：EKF 预测目标速度 + 弹簧模型跟踪 |
| Controller | `pri_adaptive_mppi::PriAdaptiveMppi` | 自适应 MPPI：normal/uphill/downhill 三模式切换 |
| BT | `tracking_bt.xml` | 自定义行为树，含 `GoalUpdater` + `PlannerSelector` |

ElasticTracker 关键参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `tracking_dist` | 1.0 m | 期望跟踪距离（弹簧平衡位置） |
| `tracking_dur` | 3.0 s | 预测/规划时域 |
| `tracking_dt` | 0.2 s | 预测时间步长 |
| `use_tracking` | true | 启用目标追踪（内部 10Hz 重规划） |
| `ekf_enabled` | true | EKF 估计目标速度 |
| `target_timeout` | 1.0 s | 目标丢失超时，超时后悬停 |
| `cost_weight` | 5.0 | A* 代价权重 |
| `tracking_zone_vertices` | 多边形 | 追踪区域约束顶点 |
| `target_topic` | `/detected_target_pose` | 目标位置订阅话题 |

PriAdaptiveMppi 的三种模式：

| 模式 | 触发条件 | 用途 |
|------|----------|------|
| `normal` | 默认 | 常规导航/追踪 |
| `uphill` | 上坡→下坡穿越直线膨胀区 | 上坡方向通过 |
| `downhill` | 下坡→上坡穿越直线膨胀区 | 下坡方向通过 |

### 自适应 MPPI 测试配置 — `config/test_adaptive_mppi_params.yaml`

| 组件 | 插件 | 说明 |
|------|------|------|
| Planner | `nav2_smac_planner/SmacPlanner2D` | 标准混合 A* 规划器 |
| Controller | `pri_adaptive_mppi::PriAdaptiveMppi` | 自适应 MPPI（同上） |
| BT | Nav2 默认 BT | `navigate_to_pose_w_replanning_and_recovery.xml` |

## 行为树 — `tracking_bt.xml`

```
RecoveryNode (9999次重试)
├── PipelineSequence
│   ├── RateController (10Hz)
│   │   └── Sequence
│   │       ├── PlannerSelector     ← 可在 GridBased / ElasticTracker 间切换
│   │       └── GoalUpdater         ← 将 /target_goal 注入 goal
│   │           └── ComputePathToPose
│   └── Sequence
│       ├── ControllerSelector      ← 可在 FollowPath 等控制器间切换
│       └── FollowPath
└── ReactiveFallback
    ├── GoalUpdated                 ← 目标更新时触发重规划
    └── RoundRobin
        └── ClearLocalCostmap + ClearGlobalCostmap
```

与标准 Nav2 BT 的核心差异：
- **GoalUpdater**：动态更新追踪目标，无需重新发送 action goal
- **PlannerSelector**：运行时切换规划器
- **PipelineSequence**：规划完成即开始跟踪，不等路径完全执行完毕

## 编译

```bash
# 在 ROS 2 工作空间中
colcon build --packages-select sim
```

该包仅安装 launch/config/urdf 等文件，无 C++ 编译。

依赖：
- `gazebo_ros` — Gazebo 仿真
- `xacro` — URDF 预处理
- `rviz2` — 可视化
- `nav2_bringup` — Nav2 导航栈
- `elastic_tracker_nav2` — 弹性追踪规划器 + 自适应 MPPI 控制器（自定义包）
- `teleop_twist_keyboard` — 键盘遥控

## RViz 布局

预配置的 [rviz/sim.rviz](rviz/sim.rviz) 包含以下显示面板：

| 显示项 | 话题 | 说明 |
|--------|------|------|
| Grid | — | 地图网格参考线 |
| TF | — | 坐标系树 (map → odom → base_link) |
| Map | `/map` | 静态地图 |
| LaserScan | `/scan` | 激光雷达点云 (红色) |
| RobotModel | `/robot_description` | 机器人 3D 模型 |
| Global Costmap | `/global_costmap/costmap` | 全局代价地图 |
| Local Costmap | `/local_costmap/costmap` | 局部代价地图 |
| Global Plan | `/plan` | 全局路径 (绿色) |
| Local Plan | `/local_plan` | 局部路径 (黄色) |
| TrackingZone | `/tracking_zone` | 追踪区域多边形 |
| AdaptiveLine | `/FollowPath/adaptive_line_visualization` | 自适应直线膨胀层 |

## 文件结构

```
sim/
├── CMakeLists.txt                          # ament_cmake 安装脚本
├── package.xml
├── map.pgm / map.yaml                      # 静态地图文件
├── behavior_trees/
│   └── tracking_bt.xml                     # 自定义追踪行为树
├── config/
│   ├── sim_nav2_params.yaml                # 弹性追踪 + MPPI 参数
│   └── test_adaptive_mppi_params.yaml      # 自适应 MPPI 测试参数
├── launch/
│   ├── sim.launch.py                       # 完整弹性追踪仿真
│   └── test_adaptive_mppi.launch.py        # 自适应 MPPI 测试
├── rviz/
│   └── sim.rviz                            # RViz 配置文件
├── scripts/
│   └── target_relay.py                     # 目标位姿转发 + Nav2 action 发送
├── urdf/
│   ├── omni_robot.urdf.xacro               # 追捕者 URDF (蓝色，含 LiDAR)
│   └── target.urdf.xacro                   # 目标 URDF (红色)
└── worlds/
    └── empty.world                          # 空 Gazebo 世界
```

## 已知限制

- `sim_nav2_params.yaml` 中的 `default_nav_to_pose_bt_xml` 使用硬编码绝对路径 (`/home/mac/Desktop/...`)，部署到其他机器时需修改为实际的 install 路径。
- `empty.world` 不含任何障碍物，适合验证追踪算法基本逻辑；复杂场景测试需替换 world 文件。
- Target 机器人无可视化传感器，`/detected_target_pose` 由 `target_relay` 直接读取 odom 发布（模拟理想检测）。
- 自适应 MPPI 的 `normal` / `uphill` / `downhill` 三套参数当前完全相同，实际调试时应根据场景差异化配置。

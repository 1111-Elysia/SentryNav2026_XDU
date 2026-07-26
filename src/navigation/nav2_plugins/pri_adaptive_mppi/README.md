# pri_adaptive_mppi

自适应 MPPI 控制器 Wrapper。根据全局规划路径与预定义直线的穿越方向，自动在 **普通 / 上坡 / 下坡** 三套 MPPI 参数之间切换。内部通过 pluginlib 动态加载 `nav2_mppi_controller::MPPIController`，切换模式时销毁旧实例并以新模式前缀重新配置。

**Normal 模式** 内置三段式到点控制 — 接近目标时自动限速，极近距离 P 控制直驱防止画弧绕圈（与 `GoalApproachController` 逻辑一致）。

与 `line_crossing_layer` 共享直线定义格式，实现"costmap 引导垂直穿越 + 控制器自适应参数切换"的完整跨越管线。

## 模式切换逻辑

```
每一帧 computeVelocityCommands():
  │
  ├── 机器人在膨胀区内？
  │   ├── 否 → NORMAL（默认参数）
  │   └── 是 →
  │       ├── 路径与直线相交 → 检测方向
  │       │   ├── 左→右  → UPHILL（上坡参数）
  │       │   └── 右→左  → DOWNHILL（下坡参数）
  │       └── 在区内但路径未穿线 → 保持当前模式
  │
  └── 多条直线重叠 → 选距离机器人最近的那条
```

**上坡/下坡方向约定**：直线方向为 `point_1 → point_2`，其**左侧**为上坡侧，**右侧**为下坡侧。路径从左侧穿到右侧 = UPHILL；从右侧穿到左侧 = DOWNHILL。

## 膨胀区

每条直线有独立的非对称膨胀区：
- **`inflation_radius_uphill`**：上坡侧（左侧）膨胀半径
- **`inflation_radius_downhill`**：下坡侧（右侧）膨胀半径

膨胀区是一个以直线为对称轴的矩形多边形。机器人在膨胀区内时才会触发模式判断。

## 插件信息

| 属性 | 值 |
|------|-----|
| 插件类 | `pri_adaptive_mppi::PriAdaptiveMppi` |
| 基类 | `nav2_core::Controller` |
| 插件名（yaml 配置） | `pri_adaptive_mppi::PriAdaptiveMppi` |

## 参数

### Wrapper 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `inner_plugin` | string | `nav2_mppi_controller::MPPIController` | 内部控制器插件名 |
| `max_path_age` | float | 2.0 | 路径最大有效期 (s)，超时后不做穿越检测 |
| `uphill_braking_distance` | float | 0.5 | 上坡急停距离 (m)，见下方上坡急停说明 |
| `uphill_stuck_vw` | float | 0.5 | 上坡阶段持续输出的 Vw 值 (rad/s，发布到 `/vw`) |
| `approach_distance` | float | 1.5 | 接近区距离 (m)，dist < 此值时合速度钳位到 approach_velocity（仅 normal 模式） |
| `approach_velocity` | float | 0.5 | 接近阶段最大合速度 (m/s)（仅 normal 模式） |
| `direct_approach_distance` | float | 0.5 | 直驱模式距离 (m)，dist < 此值时 P 控制器直驱，角速度归零（仅 normal 模式） |
| `direct_approach_kp` | float | 1.0 | P 控制器增益（仅 normal 模式） |
| `lines` | string[] | `[]` | 直线名称列表 |

### 直线定义（每条直线以 `<name>.*` 配置）

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `<name>.point_1` | float[2] | `[0.0, 0.0]` | 端点 1 坐标 |
| `<name>.point_2` | float[2] | `[5.0, 0.0]` | 端点 2 坐标 |
| `<name>.inflation_radius_uphill` | float | 1.0 | 上坡侧（左）膨胀半径 (m) |
| `<name>.inflation_radius_downhill` | float | 1.0 | 下坡侧（右）膨胀半径 (m) |
| `<name>.enable_braking` | bool | false | 是否启用上坡急停 |

### 三套 MPPI 参数前缀

| 前缀 | 使用场景 | 配置方式 |
|------|---------|---------|
| `normal.*` | 默认模式，未进入膨胀区 | 与标准 MPPI 参数相同 |
| `uphill.*` | 上坡穿越（左→右） | 独立 MPPI 参数集 |
| `downhill.*` | 下坡穿越（右→左） | 独立 MPPI 参数集 |

三套参数各自独立，例如 `normal.time_steps`、`uphill.time_steps`、`downhill.time_steps` 可以不同。

## 附加行为

### 上坡 Vw 输出

在 UPHILL 模式下，每帧向 `/vw` 话题发布 `sentry_msgs/Vw` 消息（vw=uphill_stuck_vw），配合下游节点实现卡住检测与脱困。

### 上坡急停

当某条直线启用了 `enable_braking: true` 且处于 UPHILL 模式时，机器人越过直线进入下坡侧后，速度按距离线性衰减：

- 在下坡侧距离 `[0, uphill_braking_distance]` 范围内：`scale = d / braking_distance`
- 超出 `uphill_braking_distance`：不刹车

线速度和角速度等比缩放，实现平滑停车。

### Normal 模式三段式到点控制

仅在 **Normal 模式**（未进入任何膨胀区，即常规平地导航）下生效。接近目标时自动干预速度指令，防止 MPPI 在目标点附近画弧绕圈或冲过头。

```
dist = 当前位置到目标点的欧氏距离

dist > approach_distance (0.88m):
  → 完全透传 MPPI 输出，不做任何干预

direct_approach_distance (0.1m) < dist < approach_distance (0.88m):
  → 合速度钳位到 approach_velocity (0.2 m/s)，角速度等比缩放

dist < direct_approach_distance (0.1m):
  → P 控制器直驱目标点 (Kp=direct_approach_kp)，角速度强制归零
  → 目标距离 < 0.01m 时停止
```

**参数**：`approach_distance`、`approach_velocity`、`direct_approach_distance`、`direct_approach_kp`。均为 Wrapper 级参数，放在 `AdaptiveMppi.*` 顶层（非 `normal.*` 前缀下）。

**注意**：Uphill / Downhill 模式下此逻辑不生效，保持坡道穿越的完整自由度。

## RViz 可视化

本插件自动发布以下 Marker（话题：`<plugin_name>/adaptive_line_visualization`）：

| Marker | 类型 | 说明 |
|--------|------|------|
| 直线 | LINE_STRIP | 白色=非活跃, 红色=上坡, 蓝色=下坡, 灰色=其他 |
| 上坡膨胀区 | LINE_LIST | 黄色半透明矩形框 |
| 下坡膨胀区 | LINE_LIST | 黄色半透明矩形框 |

活跃直线会加粗并变色，RViz 中添加 Marker 话题即可实时观察模式切换。

### RViz 仪表盘面板

本包包含一个 RViz 面板插件 `AdaptiveMppiPanel`，在 RViz 中显示：

- 当前模式 (NORMAL / UPHILL / DOWNHILL)，颜色编码（绿/橙/红）
- vx / vy 数值
- 合速度进度条

订阅 `/FollowPath/dashboard` 话题（CSV 格式：`MODE,vx,vy`）。

## 在 Nav2 中使用

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]

    FollowPath:
      plugin: "pri_adaptive_mppi::PriAdaptiveMppi"
      inner_plugin: "nav2_mppi_controller::MPPIController"
      max_path_age: 2.0
      uphill_braking_distance: 0.5
      uphill_stuck_vw: 0.5

      # ── 到点直接控制（仅 normal 模式）──
      approach_distance: 0.88
      approach_velocity: 0.2
      direct_approach_distance: 0.1
      direct_approach_kp: 10.0

      lines: ["ramp_a"]

      ramp_a.point_1: [0.0, 0.0]
      ramp_a.point_2: [5.0, 0.0]
      ramp_a.inflation_radius_uphill: 1.0
      ramp_a.inflation_radius_downhill: 1.0
      ramp_a.enable_braking: true

      # ── normal 模式 MPPI 参数（默认）──
      normal.time_steps: 80
      normal.batch_size: 1500
      normal.motion_model: "Omni"
      # ... 其余 MPPI 参数 ...

      # ── uphill 模式 MPPI 参数 ──
      uphill.time_steps: 80
      uphill.batch_size: 1500
      # ... (可与 normal 不同) ...

      # ── downhill 模式 MPPI 参数 ──
      downhill.time_steps: 80
      downhill.batch_size: 1500
      # ... (可与 normal 不同) ...
```

## 话题

| 话题 | 方向 | 类型 | 说明 |
|------|------|------|------|
| `<name>/adaptive_line_visualization` | 发布 | `Marker` | 直线 + 膨胀区可视化 |
| `<name>/dashboard` | 发布 | `String` | 仪表盘数据 (CSV: `MODE,vx,vy`) |
| `/vw` | 发布 | `sentry_msgs/Vw` | 上坡阶段 Vw 输出 |

## 编译

```bash
colcon build --packages-select pri_adaptive_mppi
```

依赖：
- `rclcpp` / `rclcpp_lifecycle` / `pluginlib`
- `nav2_core` / `nav2_costmap_2d` / `nav2_util` / `nav2_mppi_controller`
- `tf2_ros` / `visualization_msgs` / `sentry_msgs`
- `rviz_common` + `Qt5 Widgets`（RViz 面板插件）

## 文件结构

```
pri_adaptive_mppi/
├── CMakeLists.txt
├── package.xml
├── pri_adaptive_mppi_plugin.xml         # Nav2 Controller pluginlib 导出
├── adaptive_mppi_panel_plugin.xml        # RViz Panel pluginlib 导出
├── include/pri_adaptive_mppi/
│   ├── pri_adaptive_mppi.hpp             # 主控制器 + 几何工具函数 + LineConfig
│   └── adaptive_mppi_panel.hpp           # RViz 仪表盘面板
└── src/
    ├── pri_adaptive_mppi.cpp             # 主控制器实现
    └── adaptive_mppi_panel.cpp           # RViz 面板实现
```

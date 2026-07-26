# sentry_navigation

哨兵机器人导航集成包。将 LiDAR-惯性里程计（Super-LIO / Fast-LIO / Lightning-LM）与 Nav2 导航栈桥接，负责 TF 坐标树管理、里程计发布、重定位静态变换计算、以及导航系统的有序启动。是感知输出与导航决策之间的核心胶水层。

## 架构概览

```
┌─────────────────────────────────────────────────────────┐
│  Super-LIO / Lightning-LM                                │
│    │                                                     │
│    ├── /lio/robo/odom  (E: odom → livox_frame_two)      │
│    ├── map_reloc → livox_frame_reloc  TF (D, 重定位)     │
│    └── /confidence_lightninglm  (匹配置信度)             │
│                         │                               │
│  ┌──────────────────────▼────────────────────────────┐   │
│  │         sentry_navigation                         │   │
│  │                                                   │   │
│  │  tf_odom_publisher  ──→  TF + /odom 发布          │   │
│  │  tf_monitor.py      ──→  启动条件守卫              │   │
│  │  navigation_launch  ──→  有序启动编排              │   │
│  └──────────────────────┬────────────────────────────┘   │
│                         │                               │
│  ┌──────────────────────▼────────────────────────────┐   │
│  │  Nav2 Stack                                       │   │
│  │  MPPI Controller · SmacHybrid Planner · BT Nav    │   │
│  └───────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────┘
```

## TF 坐标树

### 本包发布的 TF 链

```
map ──(A, static)──→ odom ──(B, dynamic)──→ base_link ──(C, static)──→ livox_frame
                                                      │
                                                      └──(static)──→ odin1_base_link
```

### 外部输入的变换（用于推导 A 和 B）

```
map_reloc ──(D, SLAM重定位)──→ livox_frame_reloc
   odom    ──(E, /lio/robo/odom)──→ livox_frame_two
```

| 变换 | 名称 | 类型 | 来源 | 说明 |
|------|------|------|------|------|
| A | `map → odom` | static | 启动时从 D 平均计算 | 将 odom 系对齐到地图系 |
| B | `odom → base_link` | dynamic | 从 `/lio/robo/odom` (E) 计算 | 机器人实时位姿 |
| C | `base_link → livox_frame` | static | 参数配置 | LiDAR 安装外参 |
| D | `map_reloc → livox_frame_reloc` | dynamic | SLAM 重定位模块 | 用于计算静态 A |
| E | `odom → livox_frame_two` | dynamic | `/lio/robo/odom` | LIO 里程计原始输出 |

### 变换关系

```
B = E * C_inv          (odom → base_link 从 LIO 里程计推导)
A = avg(D) * C_inv     (map → odom 从重定位 D 的平均值计算)
```

其中 `C_inv` 是 `livox_frame → base_link`，即将 LiDAR 系下的位姿转换到 base_link 系。

## 组件

### 1. tf_odom_publisher — 重定位模式

**可执行文件**: `tf_odom_publisher`  
**源码**: [src/tf_odom_publisher.cpp](src/tf_odom_publisher.cpp)

负责完整的 TF 树构建和里程计发布：

1. **发布静态 C** (`base_link → livox_frame`)：启动时立即发布
2. **收集 D 样本**：启动后监听 `map_reloc → livox_frame_reloc`，收集 `a_collect_duration_sec` 秒，通过 `/confidence_lightninglm` 置信度过滤
3. **计算并发布静态 A** (`map → odom`)：对 D 样本做平移平均 + 四元数半球安全平均，Z 固定为 0，仅保留 yaw 旋转
4. **动态发布 B 和 `/odom`**：订阅 `/lio/robo/odom` (E)，计算 `B = E * C_inv`，发布 TF 和 `/odom` 话题，同时将 LiDAR 系速度转换到 base_link 系

**置信度过滤**：在收集 D 样本时，使用 `/confidence_lightninglm`（`Float64MultiArray`，`data[0]=timestamp, data[1]=confidence`）进行时间匹配过滤，仅保留 confidence > threshold 的样本，确保重定位静态变换的可靠性。

### 2. tf_only_odom — 纯里程计模式

**可执行文件**: `tf_only_odom`  
**源码**: [src/tf_only_odom.cpp](src/tf_only_odom.cpp)

`tf_odom_publisher` 的精简版，**无重定位功能**：

- 发布静态 A (`map → odom`) 为单位变换（map 与 odom 重合）
- 发布静态 C (`base_link → livox_frame`)
- 从 `/lio/robo/odom` 动态发布 B 和 `/odom`
- 可选：首帧归零模式（`align_base_link_to_map_on_start`），使启动位置对齐原点

适用场景：不需要重定位的简单导航测试。

### 3. odin_tf — Odin重定位模式

**可执行文件**: `odin_tf`  
**源码**: [src/odin_tf.cpp](src/odin_tf.cpp)

用于第二个 LiDAR（Odin 系列）的 TF 桥接：

- 监听 `map → odin1_base_link` TF 变换
- 利用已知的 `base_link → odin1_base_link` 外参，反向求解 `map → base_link`
- 结合 `map → odom`，发布 `odom → base_link` TF
- 订阅 `/rm_referee/game_status`（预留比赛阶段联动逻辑）

### 4. tf_monitor — TF 启动守卫

**可执行文件**: `tf_monitor.py`  
**源码**: [scripts/tf_monitor.py](scripts/tf_monitor.py)

Python 节点，在导航启动前检查 TF 链完整性：

- 必要链路：`map → odom → base_link`
- 传感器链路（二选一）：`base_link → livox_frame` 或 `odom → odin1_base_link`
- 每秒检查一次，全部就绪后节点退出（返回 0）
- launch 文件通过 `OnProcessExit` 事件监听其退出，继而启动 Nav2

## 话题

### 订阅

| 话题 | 节点 | 说明 |
|------|------|------|
| `/lio/robo/odom` | tf_odom_publisher, tf_only_odom | LIO 里程计 (E)，`nav_msgs/Odometry` |
| `/confidence_lightninglm` | tf_odom_publisher | Lightning-LM 匹配置信度，`Float64MultiArray` |
| `/rm_referee/game_status` | odin_tf | 裁判系统比赛状态（预留） |

### 发布

| 话题 | 节点 | 说明 |
|------|------|------|
| `/odom` | tf_odom_publisher, tf_only_odom | Nav2 所需里程计 |
| `/scan_mode` | odin_tf | 扫描模式切换信号（预留） |

### TF 广播

| 变换 | 类型 | 节点 |
|------|------|------|
| `base_link → livox_frame` | static | tf_odom_publisher, tf_only_odom |
| `map → odom` | static | tf_odom_publisher, tf_only_odom |
| `odom → base_link` | dynamic | tf_odom_publisher, tf_only_odom, odin_tf |

## 启动

### 标准导航启动

```bash
ros2 launch sentry_navigation navigation_launch.py
```

启动流程（时序编排）：

```
t=0    RViz（可选）
t=0    TF Monitor 启动，等待 TF 链就绪
       ↓ (TF Monitor 检测到完整 TF 链后退出)
       Map Server 启动
       ↓ (1s 后)
       Lifecycle Manager 激活 Map Server
       ↓ (2s 后)
       Nav2 Navigation Stack 启动
```

### 带参数启动

```bash
ros2 launch sentry_navigation navigation_launch.py \
  use_sim_time:=false \
  use_rviz:=true \
  map:=/path/to/map.yaml \
  params_file:=/path/to/nav2_params.yaml
```

| Launch 参数 | 默认值 | 说明 |
|-------------|--------|------|
| `use_sim_time` | `false` | 是否使用仿真时钟 |
| `use_rviz` | `false` | 是否启动 RViz |
| `map` | `bringup/map/map.yaml` | 地图 YAML 文件路径 |
| `params_file` | `bringup/config/singlenav2_params.yaml` | Nav2 参数文件路径 |
| `lidar_params_file` | `bringup/config/lidar.yaml` | LiDAR TF 参数文件路径 |
| `rviz_config` | `bringup/config/nav2_default_view.rviz` | RViz 配置文件路径 |

### 单独启动各节点

```bash
# 完整 TF/里程计发布器（含重定位）
ros2 run sentry_navigation tf_odom_publisher --ros-args --params-file config/lidar.yaml

# 简化 TF/里程计发布器（无重定位）
ros2 run sentry_navigation tf_only_odom --ros-args --params-file config/lidar.yaml

# Odin 辅助 TF 节点
ros2 run sentry_navigation odin_tf --ros-args --params-file config/odin.yaml
```

## 配置参数

所有参数文件统一由 `bringup` 包管理，本包 launch 默认从 `bringup/config/` 加载。各文件用途：

| 参数文件 | 对应节点 | 用途 |
|---------|---------|------|
| `bringup/config/lidar.yaml` | `tf_odom_publisher`, `tf_only_odom` | LiDAR 外参 + 里程计发布 + 重定位参数 |
| `bringup/config/odin.yaml` | `odin_tf` | Odin LiDAR 外参 |
| `bringup/config/singlenav2_params.yaml` | Nav2 全部节点 | 导航栈参数（由 launch 参数 `params_file` 指定） |

各文件内参数的具体含义见文件内注释。

## 编译

```bash
colcon build --packages-select sentry_navigation
```

依赖：
- `rclcpp` / `rclpy`
- `tf2_ros` / `tf2_geometry_msgs` / `tf2_msgs`
- `nav2_bringup` / `nav2_msgs`
- `rm_referee_msgs`
- `geometry_msgs` / `nav_msgs` / `sensor_msgs`

## 文件结构

```
sentry_navigation/
├── CMakeLists.txt                     # ament_cmake 构建脚本
├── package.xml
├── config/
│   ├── lidar.yaml                     # LiDAR TF 外参 + 里程计发布参数
│   ├── odin.yaml                      # Odin 辅助 LiDAR TF 参数
│   ├── navigation_params.yaml         # Nav2 导航栈完整参数
│   └── nav2_default_view.rviz         # RViz 预配置
├── launch/
│   └── navigation_launch.py           # 有序启动编排 launch
├── scripts/
│   └── tf_monitor.py                  # TF 链完整性检查守卫
└── src/
    ├── tf_odom_publisher.cpp          # 核心 TF/里程计发布器（含重定位）
    ├── tf_only_odom.cpp               # 简化 TF/里程计发布器（无重定位）
    └── odin_tf.cpp                    # Odin 辅助 LiDAR TF 桥接
```

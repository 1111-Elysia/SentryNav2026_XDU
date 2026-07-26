# SentryNav 2026 XDU

> [!WARNING]
> 本项目针对 **2026 赛季哨兵机器人** 进行了大量针对性设计（大小 yaw 双轴构型、正六边形高地斜坡越线、己方位置 relay 等），含有部分不通用功能，**并不适合零基础的导航新手学习**。若你是 Nav2 初学者，建议先参考 [Nav2 官方教程](https://navigation.ros.org/) 或下方开源参考中的通用项目。

西安电子科技大学 RoboMaster 2026 赛季哨兵机器人导航决策系统。基于 ROS 2 Humble，集成 LiDAR-惯性里程计 (Super-LIO)、Nav2 导航栈 、裁判系统通信、CAN 总线底层控制和自主行为树决策。

## 系统架构

```
┌──────────────────────────────────────────────────────────┐
│                    perception                             │
│     Super-LIO           (LiDAR-惯性里程计)                 │
│     lightning-lm        (Lightning-LM 重定位)              │
│     livox_ros_driver2   (Livox MID360 驱动)               │
│     d435_process_pcl    (D435 深度图→点云)                 │
│     cpp_lidar_filter    (LiDAR 车体点云过滤)               │
│     odin_ros_driver     (Odin 辅助 LiDAR 驱动)            │
└────────────────────────┬─────────────────────────────────┘
                         │
┌────────────────────────▼─────────────────────────────────┐
│                    navigation                             │
│     sentry_navigation  (TF 管理 + Nav2 启动编排)          │
│     nav2_plugins/                                         │
│       ├── elastic_tracker_nav2   (弹性追踪规划器)          │
│       ├── pri_adaptive_mppi      (自适应 MPPI 控制器)     │
│       ├── goal_approach_controller (接近目标限速)          │
│       ├── line_crossing_layer    (直线穿越代价层)          │
│       ├── teammate_cost_layer    (队友避让代价层)          │
│       ├── costmap_intensity      (强度过滤 costmap 层)    │
│       ├── costmap_voxel          (体素过滤 costmap 层)    │
│       └── pb_nav2_plugins        (自定义行为扩展)          │
└────────────────────────┬─────────────────────────────────┘
                         │
┌────────────────────────▼─────────────────────────────────┐
│                      decision                            │
│              sentry_nav_bt_test (行为树决策)               │
│              sentry_nav_bt_gui  (调试 GUI)                │
└────────────────────────┬─────────────────────────────────┘
                         │
┌────────────────────────▼─────────────────────────────────┐
│                    communication                           │
│     can_comm           (CAN 总线 NUC↔MCU 通信)            │
│     ground_pos_relay   (己方位置中继与坐标转换)             │
│     rm_referee_ros2    (裁判系统数据接收)                   │
│     serial_comm        (USB 串口通信，已废弃)              │
│     sentry_msgs        (自定义消息定义)                     │
└──────────────────────────────────────────────────────────┘

         tools/                          bringup/
    ┌─────────────┐              ┌─────────────────┐
    │ 仿真与调试工具 │              │  中央启动编排      │
    │              │              │  参数配置管理      │
    │ sim/tracker  │              │  启动序列控制      │
    │ map/imu/pcd  │              └─────────────────┘
    │ referee mock │
    └─────────────┘
```

## 目录结构

### perception — 感知

| 包 | 说明 |
|----|------|
| `Super-LIO` | 鲁棒高效 LiDAR-惯性里程计 (RA-L 2026) |
| `lightning-lm` | Lightning-LM 重定位与回环检测 |
| `livox_ros_driver2_humble` | Livox MID360 LiDAR ROS 2 驱动 |
| `d435_process_pcl` | Intel RealSense D435 深度图→滤波点云 |
| `cpp_lidar_filter` | 基于 PCL CropBox 的车体点云过滤 |
| `odin_ros_driver` | Odin 系列辅助 LiDAR 驱动 |

### navigation — 导航

| 包 | 说明 |
|----|------|
| `sentry_navigation` | TF 坐标树管理 + Nav2 启动编排 |
| `nav2_plugins/elastic_tracker_nav2` | 弹性追踪 GlobalPlanner (MINCO 轨迹优化) |
| `nav2_plugins/pri_adaptive_mppi` | 自适应 MPPI Controller (上坡/下坡/普通三模式) |
| `nav2_plugins/goal_approach_controller` | 接近目标时限速 Wrapper |
| `nav2_plugins/line_crossing_layer` | Costmap 直线穿越代价引导层 |
| `nav2_plugins/teammate_cost_layer` | 队友避让圆形渐变代价层 |
| `nav2_plugins/costmap_intensity` | 基于点云强度的障碍物过滤层 |
| `nav2_plugins/costmap_voxel` | PCL 体素降采样+强度过滤层 |
| `nav2_plugins/pb_nav2_plugins` | 自定义 Nav2 行为扩展 |

### communication — 通信

| 包 | 说明 |
|----|------|
| `can_comm` | CAN 总线 NUC↔MCU 双向通信 (当前使用) |
| `ground_pos_relay` | 己方位置中继与坐标转换 |
| `rm_referee_ros2` | RoboMaster 裁判系统数据解析与 Mock |
| `serial_comm` | USB 串口通信 (已废弃，保留参考) |
| `sentry_msgs` | 哨兵专用 ROS 2 自定义消息 |

### decision — 决策

| 包 | 说明 |
|----|------|
| `sentry_nav_bt_test` | 哨兵自主行为树 (导航+射击+策略) |
| `sentry_nav_bt_gui` | 行为树调试 GUI |

### bringup — 中央编排

| 包 | 说明 |
|----|------|
| `bringup` | 启动序列管理 + 所有子系统参数集中配置 |

### tools — 工具

| 包 | 说明 |
|----|------|
| `sim` | Gazebo 弹性追踪仿真环境 |
| `rm_referee_mock` | 裁判系统 Mock 组件 (rqt 插件) |
| `pcd_to_nav_map` | PCD 点云→Nav2 占据栅格地图 |
| `imu-calib` | IMU 静态安装倾角自动标定 |
| `rosbag_record` | Rosbag 录制工具 |

### GCOPTER — 外部依赖

MINCO 轨迹优化求解器，被 `elastic_tracker_nav2` 引用。

## 环境要求

- **OS**: Ubuntu 22.04
- **ROS 2**: Humble
- **C++ 标准**: C++17
- **RMW**: `rmw_cyclonedds_cpp`
- **依赖**: Eigen3, PCL ≥ 1.8, nlohmann_json, Boost, OpenCV, TBB, glog, librm (CAN 硬件库)

## 编译

```bash
cd SentryNav2026_XDU
colcon build --symlink-install
source install/setup.bash
```

## 快速启动

### 1. 完整系统启动

```bash
bash start.sh
```

### 2. SLAM 建图

```bash
ros2 run lightning run_slam_online --config ./src/bringup/config/default_livox.yaml
```

### 3. 纯定位模式

```bash
ros2 run lightning run_loc_online --config ./src/bringup/config/default_livox.yaml
```

### 4. 仿真测试

```bash
ros2 launch sim sim.launch.py
```

## 数据流

```
LiDAR (MID360) ──→ cpp_lidar_filter ──→ Super-LIO ──→ TF + /odom
                    (去车身点云)          (LIO)

D435 ──→ d435_process_pcl ──→ STVL (costmap 体素层)
        (深度图→滤波点云)

CAN ←→ can_comm ←→ /cmd_vel, /vw, /target_yaw, /detected_target_pose, ...
                  ↕
          sentry_navigation (TF 桥接)
                  ↕
          Nav2 (ElasticTracker + MPPI)
                  ↕
          sentry_nav_bt_test (行为树)

裁判系统 ──→ rm_referee_ros2 ──→ ground_pos_relay ──→ teammate_cost_layer
```

## 自定义 Nav2 插件体系

本系统在标准 Nav2 之上构建了完整的自定义插件体系：

| 插件层 | 插件 | 作用 |
|--------|------|------|
| **GlobalPlanner** | `elastic_planner::ElasticPlanner` | 动态目标追踪 (MINCO 优化) |
| **Controller** | `pri_adaptive_mppi::PriAdaptiveMppi` | 斜坡自适应三模式 MPPI |
| **Controller** | `goal_approach_controller::GoalApproachController` | 目标接近限速 |
| **Costmap Layer** | `line_crossing_layer::LineCrossingLayer` | 直线穿越代价引导 |
| **Costmap Layer** | `teammate_cost_layer::TeammateCostLayer` | 队友圆形梯度避让 |
| **Costmap Layer** | `costmap_intensity::ObstacleLayerIntensity` | 强度过滤障碍物层 |
| **Costmap Layer** | `costmap_intensity::VoxelLayerIntensity` | 强度过滤体素层 |
| **Costmap Layer** | `costmap_voxel::VoxelFilterLayer` | PCL 体素降采样+强度过滤 |
| **Behavior** | `pb_nav2_behaviors/BackUpFreeSpace` | 自由空间后退恢复 |

## 各包 README 索引

| 包 | README |
|----|--------|
| bringup | [README](src/bringup/README.md) |
| can_comm | [README](src/communication/can_comm/README.md) |
| serial_comm | [README](src/communication/serial_comm/README.md) |
| sentry_navigation | [README](src/navigation/sentry_navigation/README.md) |
| elastic_tracker_nav2 | [README](src/navigation/nav2_plugins/elastic_tracker_nav2/README.md) |
| pri_adaptive_mppi | [README](src/navigation/nav2_plugins/pri_adaptive_mppi/README.md) |
| goal_approach_controller | [README](src/navigation/nav2_plugins/goal_approach_controller/README.md) |
| line_crossing_layer | [README](src/navigation/nav2_plugins/line_crossing_layer/README.md) |
| teammate_cost_layer | [README](src/navigation/nav2_plugins/teammate_cost_layer/README.md) |
| costmap_intensity | [README](src/navigation/nav2_plugins/costmap_intensity/README.md) |
| costmap_voxel | [README](src/navigation/nav2_plugins/costmap_voxel/README.md) |
| cpp_lidar_filter | [README](src/perception/cpp_lidar_filter/README.md) |
| d435_process_pcl | [README](src/perception/d435_process_pcl/README.md) |
| Super-LIO | [README](src/perception/Super-LIO/README.md) |
| sim | [README](src/tools/sim/README.md) |
| ground_pos_relay | [README](src/communication/ground_pos_relay/README.md) |
| pcd_to_nav_map | [README](src/tools/pcd_to_nav_map/README.md) |
| imu-calib | [README](src/tools/imu-calib/README.md) |
| rm_referee_mock | [README](src/tools/rm_referee_mock/README.md) |

## 关键话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/cmd_vel` | `Twist` | Nav2 输出速度指令 |
| `/odom` | `Odometry` | 里程计 (由 tf_odom_publisher/tf_only_odom 发布) |
| `/scan` | `LaserScan` | LiDAR 转 LaserScan (由 livox_to_scan 发布) |
| `/livox/lidar_filtered` | `PointCloud2` | 过滤后 LiDAR 点云 |
| `/detected_target_pose` | `PoseStamped` | 目标位姿 (ElasticTracker 输入) |
| `/target_yaw` | `Float32` | 指令 yaw (大 yaw，NUC→MCU) |
| `/target/yaw` | `Float32` | 感知 yaw (小 yaw，MCU→NUC) |
| `/rm_referee/game_status` | `GameStatus` | 裁判系统比赛状态 |
| `/ground_pos_relay/teammate_pos_odom` | `GroundRobotPosition` | 队友 map 系位置 |

## 贡献者

| 贡献者 | 领域 |
|--------|------|
| [1111-Elysia](https://github.com/1111-Elysia) | 导航、通信、系统集成 |
| [casuallllllllll](https://github.com/casuallllllllll) | 决策 |
| [zhnnky329](https://github.com/zhnnky329) | 追击minco优化 |
| [QwQsCC](https://github.com/QwQsCC) | 己方坐标映射 |

## 开源参考

本项目基于以下开源项目构建：

| 项目 | 用途 | 许可 |
|------|------|------|
| [Super-LIO](https://github.com/Liansheng-Wang/Super-LIO) | LiDAR-惯性里程计 | GPL-3.0 |
| [Elastic-Tracker](https://github.com/ZJU-FAST-Lab/Elastic-Tracker) | 弹性追踪算法 (ICRA 2022)，MINCO 轨迹优化 | — |
| [GCOPTER](https://github.com/ZJU-FAST-Lab/GCOPTER) | MINCO 多项式轨迹求解器 | GPL-3.0 |
| [Lightning-LM](https://github.com/gaoxiang12/lightning-lm) | 激光重定位与回环检测 | — |
| [rm_referee_ros2](https://github.com/XDU-IRobot/rm_referee_ros2) | 裁判系统 ROS 2 通信协议 | Apache-2.0 |
| [cod-rm2026-navigation](https://gitee.com/codnavgation/cod_-rm2026_-navigation) | RM 哨兵导航参考 | — |
| [SCURM_SentryNavigation](https://github.com/PolarisXQ/SCURM_SentryNavigation) | RM 哨兵导航参考 | — |
| [pb2025_sentry_nav](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav) | RM 哨兵导航参考 | — |
| [librm](https://github.com/XDU-IRobot/librm) | RoboMaster CAN 通信库 | Apache-2.0 |

---

## 联系

如有疑问可提交 [Issue](https://github.com/1111-Elysia/SentryNav2026_XDU/issues) 或联系 [1361109760@qq.com](mailto:1361109760@qq.com)。不过作者已经退役，不保证回复时效。

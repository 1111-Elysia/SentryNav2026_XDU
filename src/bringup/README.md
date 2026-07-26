# bringup

SentryNav2026 系统的中央编排包。统一管理所有子系统的**启动顺序**和**参数配置**。不包含算法逻辑，是各功能包之间的胶水层。

## 启动流程

```
ros2 launch bringup start.launch.py
```

读取 `config/sequential_nodes.yaml`，按配置的顺序和时间间隔依次启动各子系统

`start.launch.py` 是通用的 YAML 驱动顺序启动器：解析 `sequential_nodes` 列表，支持 `node`（ROS 节点）和 `execute`（外部进程）两种类型，自动解析相对路径、按 `wait_after` 间隔依次启动。可带参数启动：

```bash
ros2 launch bringup start.launch.py start_delay:=5.0 post_start_delay:=1.0
```

| Launch 参数 | 默认值 | 说明 |
|-------------|--------|------|
| `sequence_config` | `config/sequential_nodes.yaml` | 启动序列配置文件 |
| `start_delay` | `0.0` | 首个节点启动前延迟 (s) |
| `post_start_delay` | `2.0` | 每个节点启动后的默认间隔 (s)，可被 YAML 中 `wait_after` 覆盖 |

## 启停控制

### 启动

```bash
ros2 launch bringup start.launch.py
```

### 停止

在启动终端按 `Ctrl+C`，launch 树内所有节点自动终止。

### 编辑启动序列

修改 `config/sequential_nodes.yaml`：

```yaml
sequential_nodes:
  - label: My-Node                 # 标签（日志用）
    type: node                     # node 或 execute
    package: my_package
    executable: my_executable
    name: my_node_name
    parameters:                    # 可选，ROS 参数列表
      - my_param: 1.0
    wait_after: 2.0                # 启动后等待秒数
    respawn: false                 # 是否自动重生

  - label: My-Process
    type: execute
    cmd: ["ros2", "launch", "my_pkg", "my.launch.py"]
    wait_after: 0
```

## 配置文件索引

所有参数文件集中管理在 `config/` 下：

| 文件 | 用途 | 使用者 |
|------|------|--------|
| `sequential_nodes.yaml` | 系统启动序列 | `start.launch.py` |
| `nav2_params.yaml` | Nav2 导航栈完整参数 | `sentry_navigation` |
| `lidar.yaml` | LiDAR 外参 (base_link→livox_frame) + TF 发布参数 | `tf_odom_publisher`, `tf_only_odom` |
| `odin.yaml` | Odin 辅助 LiDAR 外参 | `odin_tf` |
| `can_params.yaml` | CAN 通信参数（发送+接收+target_frame+yaw_controller） | `can_comm` |
| `serial_params.yaml` | 串口通信参数（已废弃） | `serial_comm` |
| `loc_start_pose.yaml` | 红蓝双方初始定位位姿 | Lightning-LM |
| `default_livox.yaml` | Livox + Super-LIO + Lightning-LM 参数 | Super-LIO / Lightning-LM |
| `livox_to_scan.yaml` | Livox 点云→LaserScan 转换参数 | `livox_to_scan` |
| `nav2_default_view.rviz` | RViz 预配置 | `sentry_navigation` |

## 编译

```bash
colcon build --packages-select bringup
```

该包仅安装 launch / config / map / behavior_trees 文件，无 C++ 编译。

## 文件结构

```
bringup/
├── CMakeLists.txt
├── package.xml
├── README.md
├── behavior_trees/
│   ├── navigate_to_pose_w_replanning_and_recovery.xml
│   └── navigate_through_poses_w_replanning_and_recovery.xml
├── config/                              # 所有子系统参数集中管理
│   ├── sequential_nodes.yaml            # ★ 启动序列
│   ├── nav2_params.yaml                 # ★ Nav2 完整参数
│   ├── lidar.yaml
│   ├── odin.yaml
│   ├── can_params.yaml
│   ├── serial_params.yaml
│   ├── loc_start_pose.yaml
│   ├── default_livox.yaml
│   ├── livox_to_scan.yaml
│   └── nav2_default_view.rviz
├── launch/
│   └── start.launch.py                  # ★ 主启动入口
├── map/
│   ├── map.pgm / map.yaml / index.txt
│   └── global.pcd
```

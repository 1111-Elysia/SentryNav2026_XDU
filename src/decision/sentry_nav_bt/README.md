# sentry_nav_bt

`sentry_nav_bt` 是基于 ROS 2、BehaviorTree.CPP 和 Nav2 的哨兵导航决策包。节点接收裁判系统状态和 TF 位姿，从 JSON 加载命名路径点，通过行为树组织导航、巡逻、回补、复活、打符、前哨站进攻及姿态控制。

默认启动树为：

```text
config/bt/uc_adaptive_training.xml
```

## 运行结构

```text
/rm_referee/* ─┐
map -> base_link ─┼─> 黑板与局内状态 ─> BehaviorTree.CPP ─> NavigateToPose
waypoints.json ──┘                    ├─> /rm_referee/tx
                                     ├─> /scan_mod_type
                                     ├─> /auto_shoot_type
                                     ├─> /yaw_controller
                                     ├─> /outpost_mode_type
                                     └─> /vw
```

启动后，节点会：

1. 加载路径点和行为树 XML；
2. 用 `init` 路径点向 `/initialpose` 发布初始位姿；
3. 等待 `navigate_to_pose` action server；
4. 监听裁判系统和 TF，并持续更新黑板；
5. 创建并循环 tick 行为树；
6. 启用 Groot2 实时监视。

## 依赖与编译

主要依赖：

- ROS 2 Humble
- BehaviorTree.CPP
- Nav2
- `rclcpp`、`rclcpp_action`
- `geometry_msgs`
- `tf2`、`tf2_ros`、`tf2_geometry_msgs`
- `nlohmann_json`
- `sentry_msgs`
- `rm_referee_msgs`

在工作空间根目录执行：

```bash
colcon build --packages-up-to sentry_nav_bt
source install/setup.bash
```

## 启动

启动默认的适应性训练树：

```bash
ros2 launch sentry_nav_bt sentry_nav_bt.launch.py
```

仅检查 XML、子树和注册节点，不执行导航：

```bash
ros2 launch sentry_nav_bt sentry_nav_bt.launch.py \
  validate_bt_only:=true
```

选择其他主树：

```bash
ros2 launch sentry_nav_bt sentry_nav_bt.launch.py \
  bt_xml_filename:="$(ros2 pkg prefix sentry_nav_bt)/share/sentry_nav_bt/config/bt/uc_patrol.xml"
```

启动供 `sentry_nav_bt_gui` 控制的单点导航树：

```bash
ros2 launch sentry_nav_bt simple_nav.launch.py
```

## 启动参数

### `sentry_nav_bt.launch.py`

| 参数 | 默认值 | 说明 |
|---|---|---|
| `bt_xml_filename` | `config/bt/uc_adaptive_training.xml` | 主行为树 XML |
| `bt_main_tree_id` | `MainTree` | 执行的主树 ID |
| `waypoints_file` | `config/waypoints.json` | 路径点文件 |
| `bt_subtree_dir` | `config/bt/modules` | 子树目录 |
| `validate_bt_only` | `false` | 只校验行为树 |
| `save_terminal_log` | `true` | 保存完整 stdout/stderr |
| `terminal_log_dir` | 启动目录下 `src/decision/sentry_nav_bt/logs` | 终端日志根目录 |

`terminal_log_dir` 使用启动 `start.sh` 或执行 `ros2 launch` 时的当前工作目录。默认情况下，每次启动会在该目录下建立带时间和 PID 的运行目录，并写入 `launch.log`。

关闭完整终端日志：

```bash
ros2 launch sentry_nav_bt sentry_nav_bt.launch.py \
  save_terminal_log:=false
```

指定日志目录：

```bash
ros2 launch sentry_nav_bt sentry_nav_bt.launch.py \
  terminal_log_dir:=/tmp/sentry_bt_logs
```

### `simple_nav.launch.py`

该入口固定使用 `simple_nav.xml`，支持 `validate_bt_only`，路径点和子树从本包安装目录加载。

## 行为树

主树位于 `config/bt/`，可复用子树位于 `config/bt/modules/`。

| 主树 | 用途 |
|---|---|
| `uc_adaptive_training.xml` | 默认适应性训练流程；在打符、回补等阶段之间执行 `myhero` 与梯形高地往返巡逻 |
| `uc_myhero.xml` | 英雄高地驻守策略 |
| `uc_fortress.xml` | 堡垒驻守策略 |
| `uc_chase.xml` | 高地追击策略 |
| `uc_patrol.xml` | 五个命名点的随机巡逻 |
| `simple_nav.xml` | GUI 动态配置的一次性单点任务 |
| `train.xml` | 随机巡逻子树的独立训练入口 |
| `5.3tree.xml` | 保留的历史测试树 |

### 适应性训练

默认树沿用 UC 比赛阶段、复活、发射机构恢复、回补、小符、大符和前哨站处理。在巡逻时间窗中调用 `AdaptiveTrainingShuttle`，在以下两个命名点之间往返：

- `myhero`
- `trapezoid_highland`

到点后停留 5 秒再前往另一点。

### 随机巡逻

`RandomWaypointNavigate` 子树内部定义五个点：

```text
point1 point2 point3 point4 point5
```

每次随机选择一个点，完成导航后停留 5 秒。`random_wait_time_threshold` 用于组成 `stage_remain_time > threshold` 条件。在 `train.xml` 中传入 `0` 仍然有作用：比赛剩余时间大于 0 时持续巡逻，归零时退出。

主树只负责调用该子树，不再重复定义五个巡逻点或到点等待流程。

### 单点导航

`simple_nav.xml` 从运行时参数取得目标、姿态和阈值。比赛开始后执行一次任务，完成后保持完成状态；参数被成功更新时会清除完成标记并执行新任务。

配套界面见 `sentry_nav_bt_gui`。

## 路径点

路径点文件为 `config/waypoints.json`：

```json
{
  "waypoints": [
    {
      "name": "init",
      "x": 0.0,
      "y": 0.0,
      "yaw": 0.0
    }
  ]
}
```

每个点必须包含唯一的 `name` 以及 `x`、`y`、`yaw`。加载后会写入：

- `waypoint_0`、`waypoint_1` 等顺序键；
- `waypoint_<name>` 命名键；
- `waypoints` 和 `waypoints_count`。

初始位姿按 `init`、第一个路径点、`start` 的顺序查找。默认树还会使用 `supply_point`、符点、前哨站、堡垒、基地防守、高地和巡逻点等命名点。

## 单点导航运行时参数

| 参数 | 默认值 | 校验 |
|---|---:|---|
| `runtime_goal_name` | `init` | 非空；使用命名点时必须存在 |
| `runtime_use_custom_pose` | `false` | `true` 时使用临时坐标 |
| `runtime_goal_x` | `0.0` | 有限数值 |
| `runtime_goal_y` | `0.0` | 有限数值 |
| `runtime_goal_yaw` | `0.0` | 有限数值 |
| `runtime_move_posture` | `3` | 1～6 |
| `runtime_wait_posture` | `1` | 1～6 |
| `runtime_reach_threshold` | `0.25` | 大于 0，单位 m |
| `runtime_wait_time_threshold` | `5.0` | 不小于 0，单位 s |

参数通过 `/sentry_nav_bt/set_parameters_atomically` 更新。所有值校验成功后才会同时生效。

## 自定义行为树节点

| 类别 | 节点 |
|---|---|
| 导航与路径点 | `ReliableNavigateToPose`、`GoalSelector`、`PatrolGoalSelector`、`RandomSelector`、`CheckGoalReached` |
| 黑板与条件 | `SetBlackboardValue`、`CheckCondition`、`CompareValues`、`PrintNode`、`PrintBlackboardValue` |
| 裁判系统动作 | `MaintainSentryPosture`、`ResolveSentryPosture`、`ConfirmResurrection`、`BuySentryProjectile`、`EngageRune`、`EngageOutpost` |
| 控制话题 | `PublishScanMode`、`PublishVw` |
| 其他 | `Wait`、`UseTrackingPlanner` |

`EngageRune` 在到达符点并开始请求前先发布 yaw 控制；当前从 yaw 发出到能量机关激活请求的等待时间为 3 秒。

## ROS 接口

主要输入：

- `navigate_to_pose`
- TF `map -> base_link`
- `/rm_referee/game_status`
- `/rm_referee/game_robot_hp`
- `/rm_referee/event_data`
- `/rm_referee/robot_status`
- `/rm_referee/power_heat_data`
- `/rm_referee/projectile_allowance`
- `/rm_referee/rfid_status`
- `/rm_referee/sentry_info`
- `/rm_referee/hurt_data`
- `/rm_referee/robot_pos`
- `/rm_referee/ground_robot_position`
- `/rm_referee/map_command`

主要输出和调用：

| 接口 | 用途 |
|---|---|
| `/initialpose` | 发布初始位姿 |
| `/rm_referee/tx` | 姿态、复活、买弹、能量机关等裁判系统指令 |
| `/scan_mod_type` | 视觉扫描模式 |
| `/auto_shoot_type` | 自动射击开关 |
| `/yaw_controller` | 打符云台转向 |
| `/outpost_mode_type` | 前哨站模式 |
| `/vw` | 特定任务的底盘控制标志 |

## Groot2

节点创建 `BT::Groot2Publisher`，可用 Groot2 连接运行中的行为树。项目文件为：

```text
config/bt/sentry_nav_bt.btproj
```

# sentry_nav_bt_test

`sentry_nav_bt_test` 是一个基于 `BehaviorTree.CPP + Nav2 + ROS 2` 的哨兵导航包。它把裁判系统状态、TF 位姿、路径点配置和自定义行为树节点组织到同一个执行入口里，用来驱动哨兵在比赛中的导航与局内决策。

当前默认启动入口为 `launch/sentry_nav_bt_test.launch.py`，默认加载的行为树为 `config/ul.xml`。

## 功能概览

- 等待 `navigate_to_pose` action server 就绪后再开始执行行为树
- 监听 `/rm_referee/*` 相关话题，并将比赛状态、血量、受击信息、中心增益点占领状态等写入黑板
- 根据机器人 ID 自动选择红蓝方路径点文件
  - `robot_id == 7` 认为是红方
  - `robot_id == 107` 认为是蓝方
  - 如果 10 秒内没有收到机器人 ID，会退回到默认红方 ID `7`
- 从路径点配置中提取初始位姿并发布到 `/initialpose`
- 提供一组自定义行为树节点，用于目标点选择、可靠导航、巡逻、追击、自定义条件判断和裁判系统指令发送
- 默认 `ul.xml` 实现了赛前等待、比赛开始初始化、低血量回补、中心点/备用点切换与驻守逻辑

## 目录结构

```text
src/sentry_nav_bt_test
├── config
│   ├── ul.xml
│   ├── chase_bt.xml
│   ├── uc.xml
│   ├── uc_demo.xml
│   ├── ul_test.xml
│   ├── ul_3.21.xml
│   ├── waypoints_red.json
│   └── waypoints_blue.json
├── include/sentry_nav_bt_test
│   ├── topic_listener.hpp
│   ├── reliable_navigate_to_pose.hpp
│   ├── chase_target_action.hpp
│   ├── patrol_nodes.hpp
│   └── ...
├── launch
│   └── sentry_nav_bt_test.launch.py
├── scripts
│   └── compatibility_bridge.py
└── src
    ├── sentry_nav_bt_test.cpp
    ├── reliable_navigate_to_pose.cpp
    ├── chase_target_action.cpp
    ├── patrol_nodes.cpp
    └── ...
```

## 依赖

编译和运行至少需要以下组件可用：

- `rclcpp`
- `rclcpp_action`
- `nav2_msgs`
- `nav2_behavior_tree`
- `nav2_util`
- `behaviortree_cpp_v3`
- `geometry_msgs`
- `nav_msgs`
- `tf2`
- `tf2_ros`
- `tf2_geometry_msgs`
- `nlohmann_json`
- `sentry_msgs`
- `rm_referee_msgs`
- `rm2_referee_msgs`

其中 `rm2_referee_msgs` 主要用于旧协议兼容桥接脚本；如果你不启用 `use_old_protocol:=true`，它不会进入主逻辑，但仍然需要包能被导入。

## 编译

在工作空间根目录执行：

```bash
colcon build --packages-up-to sentry_nav_bt_test
source install/setup.bash
```

如果依赖包也在同一个工作空间里，建议先保证它们已经成功编译。

## 快速开始

### 1. 默认启动

```bash
ros2 launch sentry_nav_bt_test sentry_nav_bt_test.launch.py
```

默认行为：

- 加载 `config/ul.xml`
- 红方路径点文件使用 `config/waypoints_red.json`
- 蓝方路径点文件使用 `config/waypoints_blue.json`
- 节点参数中固定设置了 `use_sim_time=True`

### 2. 使用旧协议桥接

如果上游仍然发布的是 `/rm2_referee/*` 话题，可以开启兼容桥接：

```bash
ros2 launch sentry_nav_bt_test sentry_nav_bt_test.launch.py \
  use_old_protocol:=true \
  team_color:=red
```

此时会额外启动 `scripts/compatibility_bridge.py`，把旧协议话题桥接到本包使用的新协议命名空间 `/rm_referee/*`。

### 3. 切换到追击行为树

```bash
ros2 launch sentry_nav_bt_test sentry_nav_bt_test.launch.py \
  bt_xml_filename:=$(ros2 pkg prefix sentry_nav_bt_test)/share/sentry_nav_bt_test/config/chase_bt.xml
```

`chase_bt.xml` 会启用 `ChaseTarget` 节点，默认订阅 `/autoaim/target_bl`，并持续向 `navigate_to_pose` 发送站位追击目标。

## 启动参数

`launch/sentry_nav_bt_test.launch.py` 当前暴露了以下启动参数：

- `bt_xml_filename`
  - 行为树 XML 完整路径
  - 默认值：安装目录下的 `config/ul.xml`
- `waypoints_red_file`
  - 红方路径点 JSON
- `waypoints_blue_file`
  - 蓝方路径点 JSON
- `bt_message_log_file`
  - `PrintNode` 额外落盘的日志文件路径
  - 默认值：`/tmp/sentry_nav_bt_messages.log`
  - 传空字符串可关闭该文件输出
- `use_old_protocol`
  - 是否启动旧协议到新协议的桥接
  - 默认值：`false`
- `team_color`
  - 仅兼容桥接脚本使用
  - 可选值：`red` / `blue`

注意：

- 当前 launch 文件里把 `navigate_bt_node` 的 `use_sim_time` 写死成了 `True`
- 如果你在真机上运行，通常需要把它改成 `False`，或者把 launch 文件改成可配置参数

## 行为树配置说明

### 默认 `ul.xml`

默认树大致分为四段：

1. 等待裁判系统连接
2. 比赛未开始时持续复位局内状态
3. 比赛开始后初始化黑板变量
4. 进入局内逻辑
   - 低血量时前往 `supply_point` 回补
   - 否则根据中心增益点占领状态，在 `center_point` 和 `fallback_point` 之间切换
   - 到达中心相关点位后进入驻守
   - 驻守期间会持续发布 `/vw`

默认实现里有一个需要特别注意的点：

- `ul_center_arrive_distance_threshold` 当前默认是 `0.10 m`
- `ul_center_hold_distance_threshold` 当前默认是 `0.50 m`
- “前往中心相关点位”分支使用更小的 `arrive` 阈值判定真正回中
- “中心驻守”分支使用更大的 `hold` 半径判定是否还允许继续驻守
- 一旦偏离超过驻守半径，行为树会先把 `ul_center_ready` 置回 `0`，再重新走“前往中心相关点位”分支

这样做的目的，是把“是否需要重新回中”和“回中到什么程度才算真正到点”拆成两套语义。

`ul.xml` 依赖以下命名路径点存在：

- `init`
- `supply_point`
- `center_point`
- `fallback_point`

### 追击树 `chase_bt.xml`

追击树当前主要使用一个 `ChaseTarget` 节点，默认参数如下：

- 自瞄输入话题：`/autoaim/target_bl`
- 导航 action：`navigate_to_pose`
- 世界坐标系：`map`
- 车体坐标系：`base_link`

当目标丢失超时、TF 变换不可用或 Nav2 连续异常时，节点会返回失败，由外层行为树切换策略。

## 路径点配置格式

路径点从 JSON 文件加载，格式如下：

```json
{
  "waypoints": [
    {
      "name": "init",
      "x": 0.0,
      "y": 0.0,
      "yaw": 0.0
    },
    {
      "name": "supply_point",
      "x": -0.3,
      "y": 0.3,
      "yaw": 0.0
    }
  ]
}
```

加载后会写入黑板：

- `waypoint_0`, `waypoint_1`, ...：按顺序索引
- `waypoint_<name>`：按名字索引
- `waypoints_count`
- `waypoints`

启动时初始位姿查找顺序为：

1. `waypoint_init`
2. `waypoint_0`
3. `waypoint_start`

所以如果你希望自动发布初始位姿，最稳妥的做法是在 JSON 中保留 `name: "init"`。

## 运行时输入输出

### 主要输入

- `navigate_to_pose` action server
- TF: `map -> base_link`
- `/rm_referee/game_status`
- `/rm_referee/robot_status`
- `/rm_referee/game_robot_hp`
- `/rm_referee/event_data`
- `/rm_referee/sentry_info`
- `/rm_referee/projectile_allowance`
- `/rm_referee/hurt_data`
- `/rm_referee/robot_pos`
- `/autoaim/target_bl`
  - 仅追击树使用

### 主要输出

- `/initialpose`
  - 启动时自动发布初始位姿
- `/vw`
  - 在默认 UL 驻守逻辑中持续发布
- `/sentry_nav_bt_test/decoded_sentry_info/*`
  - 调试用的解包结果发布

### 服务调用

- `/rm_referee/tx`
  - `SetSentryPosture`
  - `RequestActivateRune`
  - `ConfirmResurrection`

这几个裁判系统动作节点都通过该服务发送数据。

## 黑板中的关键数据

本包运行时会持续维护一批黑板键，常用的有：

- `robot_id`
- `current_hp`
- `game_progress`
- `game_status_received`
- `center_gain_point_occupancy_status`
- `current_posture`
- `can_activate_rune`
- `waypoint_now`
- `waypoint_now_valid`
- `ul_initialized`
- `ul_retreat_active`
- `ul_center_ready`
- `ul_center_goal_name`

如果你要扩展 XML，优先复用这些已有键，能少写很多桥接逻辑。

## 自定义行为树节点

本包当前注册了以下主要自定义节点：

| 节点名 | 类型 | 作用 |
| --- | --- | --- |
| `GoalSelector` | Action | 从黑板按名字取目标点 |
| `PatrolGoalSelector` | Action | 依次选择巡逻点 |
| `ReliableNavigateToPose` | Stateful Action | 带重发、到点判定和重试逻辑的导航节点 |
| `CheckGoalReached` | Condition | 基于 `waypoint_now` 判断是否到点 |
| `ChaseTarget` | Stateful Action | 把自瞄点转成站位导航目标并持续追击 |
| `CheckCondition` | Condition | 黑板值与阈值比较 |
| `CompareValues` | Condition | 两个黑板值之间比较 |
| `SetBlackboardValue` | Action | 往黑板写入常量 |
| `PrintNode` | Action | 输出日志 |
| `PrintBlackboardValue` | Action | 打印某个黑板键值 |
| `RandomSelector` | Action | 从路径点列表里随机选点 |
| `SetSentryPosture` | Action | 通过裁判系统切换哨兵姿态 |
| `RequestActivateRune` | Action | 请求激活能量机关 |
| `ConfirmResurrection` | Action | 连发确认复活指令 |
| `AutoAimAndFire` | Action | 当前为 mock 节点，仅打印日志 |

此外也注册了 Nav2 自带节点：

- `NavigateToPose`
- `Wait`

## 与 Groot 联调

节点启动后会创建 `BT::PublisherZMQ`，日志中会输出 Groot ZMQ Publisher 启动信息。可以直接用 Groot 打开：

- `bt.btproj`
- `Project.btproj`

来观察行为树执行过程。

## 常见问题

### 1. 节点启动后一直不进入主逻辑

优先检查：

- `navigate_to_pose` action server 是否已经就绪
- `/rm_referee/game_status` 是否有数据
- `/rm_referee/robot_status` 是否能收到 `robot_id`

### 2. 一直报当前位置不可用

说明 `map -> base_link` 的 TF 还没有建立，或者定位链路没有正常工作。此时：

- `waypoint_now_valid` 会变成 `false`
- `CheckGoalReached` 和 `ReliableNavigateToPose` 的到点判定会退化
- 默认 UL 驻守时的 `/vw` 保持逻辑也会暂停

### 2.1. 中心驻守的回中/驻守边界怎么理解

优先检查：

- `ul_center_arrive_distance_threshold` 当前默认值是否为 `0.10`
- `ul_center_hold_distance_threshold` 当前默认值是否为 `0.50`
- 是否希望“偏离超过驻守半径后，必须重新回到中心点附近”这一行为

当前默认树的语义是：

- 进入驻守前，要先回到更小的 `arrive` 阈值内
- 进入驻守后，只要仍在更大的 `hold` 半径内，就允许继续驻守
- 一旦偏离超过 `hold` 半径，就退出驻守并重新走回中流程

### 3. 初始位姿没有生效

检查：

- 路径点 JSON 中是否包含 `init`
- `init` 是否处于 `map` 坐标系语义下的正确位置
- 定位模块是否在监听 `/initialpose`

### 4. 旧协议模式下没有数据

检查：

- 是否开启了 `use_old_protocol:=true`
- `team_color` 是否与当前阵营一致
- `/rm2_referee/*` 原始话题是否确实存在

## 开发建议

- 如果只是改战术逻辑，优先改 `config/*.xml` 和路径点 JSON
- 如果需要增强稳定性，重点看 `ReliableNavigateToPose` 和 `topic_listener.hpp`
- 如果要接入新的裁判系统字段，优先在 `BlackboardManager` 中完成订阅和黑板映射
- 如果要扩展追击逻辑，重点看 `ChaseTargetAction`

## 当前默认行为的几个实现细节

- 导航节点等待 `navigate_to_pose` 最长 60 秒
- 获取机器人 ID 最长等待 10 秒，超时会退回红方 ID `7`
- 当前位置由 TF 定时写入黑板键 `waypoint_now`
- 中心驻守激活时会持续发布 `/vw = 1`
- 退出中心驻守后会停止继续发布 `/vw`
- `PrintNode` 消息默认会额外写入 `/tmp/sentry_nav_bt_messages.log`，可通过 `bt_message_log_file` 参数修改或置空关闭
- 默认中心到点阈值 `ul_center_arrive_distance_threshold = 0.10 m`
- 默认中心驻守半径 `ul_center_hold_distance_threshold = 0.50 m`
- `/vw` 驻守发布逻辑已从 `topic_listener.hpp` 中拆到独立的 `center_hold_vw_controller.hpp`

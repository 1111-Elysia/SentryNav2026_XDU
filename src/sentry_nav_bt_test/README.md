# sentry_nav_bt_test

`sentry_nav_bt_test` 是一个基于 `BehaviorTree.CPP + Nav2 + ROS 2` 的哨兵导航包。它把裁判系统状态、TF 位姿、路径点配置和自定义行为树节点组织到同一个执行入口里，用来驱动哨兵在比赛中的导航与局内决策。

当前默认启动入口为 `launch/sentry_nav_bt_test.launch.py`，默认加载的行为树为 `config/uc_test.xml`。

## 功能概览

- 等待 `navigate_to_pose` action server 就绪后再开始执行行为树
- 监听 `/rm_referee/*` 相关话题，并将比赛状态、血量、受击信息、中心增益点占领状态等写入黑板
- 从单一 `config/waypoints.json` 加载路径点，不再按机器人 ID 区分红蓝方点位
- 从路径点配置中提取初始位姿并发布到 `/initialpose`
- 提供一组自定义行为树节点，用于目标点选择、可靠导航、巡逻、追击、自定义条件判断和裁判系统指令发送
- 默认 `uc.xml` 实现了高校赛对抗主流程，包括赛前等待、开赛初始化、复活、回补、小符/大符激活、前哨站点位、防御姿态切换和堡垒驻守
- `ul.xml` 仍然保留了赛前等待、比赛开始初始化、低血量回补、中心点/备用点切换与驻守逻辑

## 目录结构

```text
src/sentry_nav_bt_test
├── config
│   ├── ul.xml
│   ├── chase_bt.xml
│   ├── uc.xml
│   ├── ul_3.21.xml
│   └── waypoints.json
├── include/sentry_nav_bt_test
│   ├── topic_listener.hpp
│   ├── reliable_navigate_to_pose.hpp
│   ├── chase_target_action.hpp
│   ├── patrol_nodes.hpp
│   └── ...
├── launch
│   └── sentry_nav_bt_test.launch.py
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
- `nav2_util`
- `behaviortree_cpp`
- `geometry_msgs`
- `nav_msgs`
- `tf2`
- `tf2_ros`
- `tf2_geometry_msgs`
- `nlohmann_json`
- `sentry_msgs`
- `rm_referee_msgs`

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

- 加载 `config/uc_test.xml`
- 路径点文件使用 `config/waypoints.json`
- 节点参数中固定设置了 `use_sim_time=False`

### 2. 切换到追击行为树

```bash
ros2 launch sentry_nav_bt_test sentry_nav_bt_test.launch.py \
  bt_xml_filename:=$(ros2 pkg prefix sentry_nav_bt_test)/share/sentry_nav_bt_test/config/chase_bt.xml
```

`chase_bt.xml` 会启用 `ChaseTarget` 节点，默认订阅 `/autoaim/target_bl`，并持续向 `navigate_to_pose` 发送站位追击目标。当前版本在接近目标后会进入 `HOLD` 保持态，不会直接触发真实开火。

## 启动参数

`launch/sentry_nav_bt_test.launch.py` 当前包含以下启动参数：

- `bt_xml_filename`
  - 行为树 XML 完整路径
  - 默认值：安装目录下的 `config/uc_test.xml`
- `waypoints_file`
  - 路径点 JSON
  - 默认值：安装目录下的 `config/waypoints.json`
- `bt_message_log_file`
  - `PrintNode` 额外落盘的日志目录/基准文件路径
  - 默认值：`/tmp/sentry_nav_bt_messages.log`
  - 固定路径会持续追加所有运行记录，例如：`/tmp/sentry_nav_bt_messages.log`
  - 同时也会按每次启动生成唯一文件，例如：`/tmp/sentry_nav_bt_messages_2026-03-27_21-05-33_pid12345.log`
  - 传空字符串可关闭该文件输出
注意：

- 当前 launch 文件里把 `navigate_bt_node` 的 `use_sim_time` 写死成了 `False`

## 行为树配置说明

### 可选树 `ul.xml`

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


`ul.xml` 依赖以下命名路径点存在：

- `init`
- `supply_point`
- `center_point`
- `fallback_point`

### 默认高校赛树 `uc.xml`

`uc.xml` 是当前 launch 的默认树，按高校赛对抗流程维护。它当前的主逻辑是：

1. 等待裁判系统连接成功
2. 比赛未开始时持续复位 UC 局内状态
3. 比赛开始时执行一次 UC 黑板初始化
4. 血量为 `0` 时持续确认免费复活
5. 低血或低弹时回 `supply_point` 回补
6. 其余时间按照 `stage_remain_time` 的时间带切阶段

当前时间轴按“剩余时间从大到小的有序 Fallback”实现，效果上已经是严格分段：

- `7:00-6:30`：前往 `small_rune_point`，尝试开局小符
- `6:30-5:30`：前往 `outpost_point`，到点后等待阶段切换
- `5:30-5:00`：再次前往 `small_rune_point`，补打小符
- `5:00-4:00`：优先前往 `outpost_point`，若己方前哨站已失则退守 `fortress`
- `4:00-3:30`：前往 `large_rune_point`，尝试激活大符
- `3:30-2:45`：在 `fortress` 驻守
- `2:45-2:15`：再次前往 `large_rune_point`，补打大符
- `2:15-1:30`：继续在 `fortress` 驻守
- `1:30-1:00`：若己方基地血量大于 `2000`，再次尝试 `large_rune_point`
- `1:00-0:00`：回 `base_defense_point` 守家


`uc.xml` 依赖以下命名路径点存在：

- `init`
- `supply_point`
- `small_rune_point`
- `large_rune_point`
- `outpost_point`
- `fortress`
- `base_defense_point`


### `uc.xml` 的到点与驻守判定

`uc.xml` 目前统一使用 `ReliableNavigateToPose` 做导航到点判定，只是不同类型点位的阈值不同：

- `supply_point`：`0.25 m`
- `small_rune_point`：`0.10 m`
- `outpost_point`：`0.25 m`
- `large_rune_point`：`0.10 m`
- `base_defense_point`：`0.25 m`
- 堡垒驻守当前分成两层
  - 回堡垒点位时，`ReliableNavigateToPose` 的到点阈值使用 `uc_fortress_hold_distance_threshold = 0.25 m`
  - 已进入驻守后，`CheckGoalReached` 允许的驻守半径使用 `uc_fortress_hold_exit_distance_threshold = 0.30 m`
- 堡垒驻守阶段仍然额外使用 `CheckGoalReached + ReliableNavigateToPose`
  - 这套逻辑和 `ul.xml` 的中心驻守是同一类思路
  - 进入堡垒驻守后，只要仍在 `0.30 m` 半径内，就继续驻守
  - 一旦偏离这个半径，就重新回堡垒点位
  - 回堡垒导航时，进入 `0.25 m` 阈值内就算重新到点
  - 堡垒驻守不再触发 `/vw = 1`，`/vw` 持续发布只保留给 UL 中心驻守

当前 `ReliableNavigateToPose` 本身在本地到点后会直接返回 `SUCCESS`，不会因为 Nav2 的异步回调稍晚一点到达就再次主动重发同一目标。联调里如果看到“已经到点但又重新开始发点”，更常见的原因不是导航节点内部死循环，而是外层行为树分支被重新进入。

## UC 当前说明

最近一轮已经处理了三个容易在 UC 联调里触发的问题：

1. `复活机制` / `回补模式` 不再在“条件尚未命中”时提前清掉 `in_rune_phase` 和 `uc_fortress_hold_active`。
   - 这样可以避免堡垒驻守状态在同一个主循环里被反复抖掉
   - 也避免了“受击姿态辅助”每 tick 都把打符态误读成非打符态

2. `MaintainSentryPosture` 现在优先读取 `current_posture`，并结合共享黑板里的最近一次姿态请求状态做去重。
   - 如果当前姿态已经是目标姿态，就直接跳过，不再重复发同样的切姿态请求
   - 如果上一次姿态请求还在全局冷却窗口内，也不会因为 UC 中另一个节点被 tick 到就再次重发
   - 默认全局冷却时间是 `posture_switch_cooldown_ms = 5000`
   - 这解决了“姿态链路有 5 秒冷却，但树里多个姿态维护节点持续反复发送”的问题

3. `EngageRune` 现在会区分几种结束语义。
   - `activated`：观察到能量机关最终进入“已激活”
   - `already_activated`：节点启动时目标符已经处于已激活
   - `window_expired`：看到“正在激活”后又回到未激活，说明窗口结束但未成功激活
   - `timeout` / `halted`：流程超时或被树打断

当前如果需要在 XML、日志或调试工具里判断打符结果，可以直接看：

- `last_rune_activation_success`
- `last_rune_activation_result`
- `last_rune_type`

当前如果要看姿态切换链路，也建议一起关注：

- `current_posture`
- `last_posture_request_target`
- `last_posture_request_pending`
- `last_posture_request_confirmed`
- `last_posture_request_result`


### 追击树 `chase_bt.xml`

追击树当前主要使用一个 `ChaseTarget` 节点，默认参数如下：

- 自瞄输入话题：`/autoaim/target_bl`
- 自瞄输入消息：`geometry_msgs/msg/Point`
- 导航 action：`navigate_to_pose`
- 世界坐标系：`map`
- 车体坐标系：`base_link`

当前实现的行为是：

- 把 `base_link` 下的自瞄点转换成带 `standoff` 的站位导航目标
- 当目标距离小于等于 `stop_dist` 时进入 `HOLD`
- 当目标重新离开到 `start_dist` 之外时恢复追击，避免在阈值附近来回抖动
- 当目标丢失超时、TF 变换不可用或 Nav2 连续异常时，节点会返回失败，由外层行为树切换策略

当前限制：

- 追击输入接口还是临时约定，尚未和自瞄侧固化正式消息定义
- `AutoAimAndFire` 仍然是 mock 节点，没有接入真实火控链路
- 追击树目前还是独立 demo，没有并入 `ul.xml` 的比赛主流程

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
  - 在 `ul.xml` 的中心驻守中持续发布
  - `EngageOutpost` 打前哨站期间会持续发布 `1`
  - `EngageOutpost` 退出、超时或被外层打断时会补发一次 `0`
- `/scan_mod_type`
  - 这是 topic 名，不是消息类型名
  - 消息类型实际是 `sentry_msgs/msg/ScanMode`
  - `EngageRune` 在满足 `can_activate_rune == 1` 且到达请求发送时机后，按顺序发送 `false`
  - `EngageRune` 成功、失败、超时或被外层打断时，会恢复 `true`
- `/auto_shoot_type`
  - `EngageRune` 在准备发送激活请求时会先发一次 `true`
  - 若后续观察到能量机关进入“正在激活”状态，会继续保持该状态
  - 流程结束时会恢复 `false`
- `/yaw_controller`
  - `EngageRune` 在准备发送激活请求时发送一次 `true`，触发云台转向能量机关
- `/outpost_mode_type`
  - `EngageOutpost` 进入打前哨站流程时发送 `true`
  - `EngageOutpost` 退出、超时或被外层打断时恢复 `false`
- `/sentry_nav_bt_test/decoded_sentry_info/*`
  - 调试用的解包结果发布

### 服务调用

- `/rm_referee/tx`
  - `MaintainSentryPosture`
  - `ConfirmResurrection`
  - `EngageRune`

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
- `uc_initialized`
- `ul_retreat_active`
- `ul_center_ready`
- `ul_center_goal_name`
- `in_rune_phase`
- `uc_fortress_hold_active`
- `uc_fortress_goal_name`
- `uc_fortress_hold_distance_threshold`
- `uc_fortress_hold_exit_distance_threshold`
- `posture_switch_cooldown_ms`
- `last_posture_request_target`
- `last_posture_request_pending`
- `last_posture_request_confirmed`
- `last_posture_request_result`
- `last_rune_activation_success`
- `last_rune_activation_result`
- `last_rune_type`


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
| `MaintainSentryPosture` | Action | 通过裁判系统维持哨兵姿态，优先读当前姿态并结合全局冷却去重 |
| `ConfirmResurrection` | Action | 连发确认复活指令 |
| `EngageRune` | Stateful Action | 管理 scan mode、yaw、激活请求和 autoshoot 的整段打符流程 |
| `EngageOutpost` | Stateful Action | 管理 scan mode、yaw、outpost mode 和 `/vw` 的前哨站进攻流程 |
| `AutoAimAndFire` | Action | 当前为 mock 节点，仅打印日志 |

此外也注册了本包内置的兼容节点：

- `Wait`

## 与 Groot 联调

节点启动后会创建 `BT::Groot2Publisher`，日志中会输出 Groot2 Publisher 启动信息。可以直接用 Groot2 打开：

- `bt.btproj`
- `Project.btproj`

来观察行为树执行过程。



## TODO

### 包结构与工程化

- 拆分 `BlackboardManager`，把裁判系统订阅、TF/当前位置更新、路径点加载、受击状态管理、调试发布等职责从 `topic_listener.hpp` 中分离
- 瘦身 `sentry_nav_bt_test.cpp` 的 `main()`，把日志初始化、机器人 ID 解析、路径点加载、初始位姿发布、行为树启动拆成独立函数或类
- 把高频使用的黑板 key、默认参数和阵营 ID 常量集中管理，减少字符串散落和 magic number
- 抽取 `PrintNode` / `PrintBlackboardValue` 共用的日志落盘逻辑，避免重复维护
- 将 `ul.xml` 继续拆成更清晰的子树或可复用片段，降低主树复杂度
- 给路径点加载、`GoalSelector`、`ReliableNavigateToPose`、`ChaseTargetAction` 增加最小测试
- 清理遗留硬编码逻辑，例如 `RandomSelector` 仍使用示例目标点，未接入实际配置
- 统一依赖与包元数据，补全 `package.xml` 里的 license，并明确哪些依赖是主流程必需、哪些仅旧协议桥接需要
- 减少大头文件中的实现代码，尽量把可以下沉到 `.cpp` 的逻辑移出头文件，优化增量编译体验
- 重新梳理启动与安装后的运行路径，降低“源码已改但运行的还是旧 install 版本”这类排查成本

### 行为树与运行逻辑

- 把赛前等待、初始化、回补、回中、驻守这些主流程的状态切换日志继续整理成更稳定、可分析的格式
- 明确 `ul.xml` 中各分支的进入条件、退出条件和关键黑板值，减少隐式耦合
- 继续收敛 `ReliableNavigateToPose` 的重发、超时、取消、结果处理语义，保证与外层行为树的状态切换一致
- 梳理“机器人 ID 获取失败时强制回退红方”这类临时策略，改成明确可配置的启动策略
- 为日志、回放、bag 分析整理固定流程，减少现场靠肉眼翻终端排查的问题

### 追击链路

- 与自瞄侧统一正式通信接口，至少明确目标点坐标系、时间戳、目标有效位、目标 ID、置信度等字段
- 为追击链路增加一层适配节点，避免 BT 直接依赖自瞄原始话题格式
- 把 `AutoAimAndFire` 从 mock 改成真实接口，明确“追到位后由谁负责瞄准/开火”
- 将 `chase_bt.xml` 中验证过的追击能力按触发条件接入 `ul.xml`，而不是长期作为独立 demo
- 明确比赛中的追击触发和退出条件，例如血量阈值、区域限制、目标稳定时间和 Nav2 连续异常次数
- 为 `ChaseTargetAction` 增加最小测试，覆盖 `HOLD/恢复追击/目标丢失/Nav2 aborted` 等关键状态切换
- 补充 rosbag 或日志回放调参流程，便于离线调整 `stop_dist`、`start_dist`、`lost_timeout`、`update_thresh`

## 当前默认行为的几个实现细节

- 导航节点等待 `navigate_to_pose` 最长 60 秒
- 获取机器人 ID 最长等待 10 秒，超时会退回红方 ID `7`
- 当前位置由 TF 定时写入黑板键 `waypoint_now`
- 中心驻守激活时会持续发布 `/vw = 1`
- 退出中心驻守后会停止继续发布 `/vw`
- 打前哨站期间会持续发布 `/vw = 1`，退出、超时或被打断时会补发一次 `/vw = 0`
- `PrintNode` 消息默认会同时写入累计历史日志 `/tmp/sentry_nav_bt_messages.log`
- `PrintNode` 消息也会额外写入按启动时间命名的独立日志文件，例如 `/tmp/sentry_nav_bt_messages_2026-03-27_21-05-33_pid12345.log`
- 每次启动都会保留自己的独立日志，同时固定路径也能看到完整运行历史
- 默认中心到点阈值 `ul_center_arrive_distance_threshold = 0.10 m`
- 默认中心驻守半径 `ul_center_hold_distance_threshold = 0.50 m`
- 默认堡垒回点阈值 `uc_fortress_hold_distance_threshold = 0.25 m`
- 默认堡垒驻守退出阈值 `uc_fortress_hold_exit_distance_threshold = 0.30 m`

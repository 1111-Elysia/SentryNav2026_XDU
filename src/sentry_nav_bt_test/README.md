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
- `uc.xml` 实现了高校赛对抗主流程，包括赛前等待、开赛初始化、复活、回补、小符/大符激活、前哨站点位、防御姿态切换和堡垒驻守

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

`chase_bt.xml` 会启用 `ChaseTarget` 节点，默认订阅 `/autoaim/target_bl`，并持续向 `navigate_to_pose` 发送站位追击目标。当前版本在接近目标后会进入 `HOLD` 保持态，不会直接触发真实开火。

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
  - `PrintNode` 额外落盘的日志目录/基准文件路径
  - 默认值：`/tmp/sentry_nav_bt_messages.log`
  - 固定路径会持续追加所有运行记录，例如：`/tmp/sentry_nav_bt_messages.log`
  - 同时也会按每次启动生成唯一文件，例如：`/tmp/sentry_nav_bt_messages_2026-03-27_21-05-33_pid12345.log`
  - 传空字符串可关闭该文件输出
- `use_old_protocol`
  - 是否启动旧协议到新协议的桥接
  - 默认值：`false`
- `team_color`
  - 仅兼容桥接脚本使用
  - 可选值：`red` / `blue`

注意：

- 当前 launch 文件里把 `navigate_bt_node` 的 `use_sim_time` 写死成了 `False`

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

### 高校赛对抗树 `uc.xml`

`uc.xml` 不是默认启动树，但已经按高校赛对抗流程维护。它当前的主逻辑是：

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

这种写法比“每个阶段同时写上下界”更鲁棒一点，原因是：

- 时间带本身仍然是严格的，前面的阶段一旦不满足，才会落到后面的阶段
- 每个时间带内部还能继续根据符是否已开、是否允许激活、前哨站是否存活等条件自动跳过不合适的任务
- 如果改成完全刚性的固定时间片，而不保留这些条件回退，现场更容易出现“当前计划不可执行，但树还在死磕该分支”的问题

所以当前实现建议保留“按剩余时间降序 + 分支内条件回退”的结构。如果后面主要诉求是可读性，我们可以再把每个阶段补成显式上下界，但不建议去掉现在这层回退鲁棒性。

`uc.xml` 依赖以下命名路径点存在：

- `init`
- `supply_point`
- `small_rune_point`
- `large_rune_point`
- `outpost_point`
- `fortress`
- `base_defense_point`

为了兼容旧配置，当前路径点 JSON 里仍保留了 `tunnel_rune_point` 和 `highway_defense` 这两个旧名字作为别名。

### `uc.xml` 的到点与驻守判定

`uc.xml` 目前统一使用 `ReliableNavigateToPose` 做导航到点判定，只是不同类型点位的阈值不同：

- 小符/大符点位默认使用 `0.10 m`
- 前哨站、补给区、守家点位默认使用 `0.25 m`
- 堡垒驻守分成两层
  - 回堡垒点位的真正到点阈值：`uc_fortress_arrive_distance_threshold = 0.10 m`
  - 已进入驻守后允许的活动半径：`uc_fortress_hold_distance_threshold = 0.25 m`
- 堡垒驻守阶段仍然额外使用 `CheckGoalReached + ReliableNavigateToPose`
  - 这套逻辑和 `ul.xml` 的中心驻守是同一类思路
  - 进入堡垒驻守后，只要仍在 `0.25 m` 半径内，就继续驻守
  - 一旦偏离这个半径，就重新回堡垒点位
  - 在堡垒驻守激活且位于驻守半径内时，会持续发布 `/vw = 1`
  - 离开驻守半径后，堡垒驻守 `/vw` 发布会停止

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
  - 在 `ul.xml` 的中心驻守和 `uc.xml` 的堡垒驻守中持续发布
- `/scan_mod_type`
  - 这是 topic 名，不是消息类型名
  - 消息类型实际是 `sentry_msgs/msg/ScanMode`
  - `EngageRune` 在打符开始时发 `false`，结束时恢复 `true`
- `/auto_shoot_type`
  - 打符流程检测到能量机关进入正在激活状态后发 `true`，结束时发 `false`
- `/yaw_controller`
  - 打符流程开始时发送一次 `true`，触发云台转向能量机关
- `/sentry_nav_bt_test/decoded_sentry_info/*`
  - 调试用的解包结果发布

### 服务调用

- `/rm_referee/tx`
  - `SetSentryPosture`
  - `RequestActivateRune`
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
- `uc_fortress_arrive_distance_threshold`
- `uc_fortress_hold_distance_threshold`

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
| `EngageRune` | Stateful Action | 管理 scan mode、yaw、激活请求和 autoshoot 的整段打符流程 |
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

### `ScanMode` 和 `/scan_mod_type` 的区别

这两个名字描述的不是同一个层级：

- `ScanMode` 是消息类型名，定义在 `sentry_msgs/msg/ScanMode.msg`
- `/scan_mod_type` 是当前工程里实际在发布/订阅的 ROS topic 名

也就是说，代码里常见的写法是：

- topic: `/scan_mod_type`
- type: `sentry_msgs::msg::ScanMode`

当前 `can_comm` 和 `serial_comm` 都已经按 `/scan_mod_type` 这个 topic 名接好了，所以如果只把 BT 侧改成 `/scanmode`，整条链路会断掉。

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
- 明确默认行为树 `ul.xml` 中各分支的进入条件、退出条件和关键黑板值，减少隐式耦合
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
- `PrintNode` 消息默认会同时写入累计历史日志 `/tmp/sentry_nav_bt_messages.log`
- `PrintNode` 消息也会额外写入按启动时间命名的独立日志文件，例如 `/tmp/sentry_nav_bt_messages_2026-03-27_21-05-33_pid12345.log`
- 每次启动都会保留自己的独立日志，同时固定路径也能看到完整运行历史
- 默认中心到点阈值 `ul_center_arrive_distance_threshold = 0.10 m`
- 默认中心驻守半径 `ul_center_hold_distance_threshold = 0.50 m`
- 默认堡垒到点阈值 `uc_fortress_arrive_distance_threshold = 0.10 m`
- 默认堡垒驻守半径 `uc_fortress_hold_distance_threshold = 0.25 m`
- `/vw` 驻守发布逻辑已从 `topic_listener.hpp` 中拆到独立的 `center_hold_vw_controller.hpp`

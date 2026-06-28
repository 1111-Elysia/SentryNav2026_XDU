# rm_referee_mock

这是 `rm_referee_ros2` 所属的一个子模块，提供了一些用于测试的 Mock 组件，用来模拟裁判系统的数据发送行为，方便在没有真实裁判系统或者不便搭建环境的情况下进行开发和测试。当前文档以《RoboMaster 2026 机甲大师高校系列赛通信协议 V2.0.0（20260626）》为准。

目前为止所有的 Mock 组件均以 rqt 插件的形式实现，编译并 source 工作空间后，在 rqt 中启动对应的插件即可使用。

如果本地 `rqt` 缓存或透视图配置容易导致启动后出现空白页，可以改用下面这个项目内启动器。它会在每次启动时自动附带 `--clear-config --force-discover`，并直接打开指定插件：

```bash
ros2 run rm_referee_mock rqt_clean_start
```

默认打开 `MatchControl`。也可以指定其他插件：

```bash
ros2 run rm_referee_mock rqt_clean_start fake_location
ros2 run rm_referee_mock rqt_clean_start keyboard
ros2 run rm_referee_mock rqt_clean_start dart_client
ros2 run rm_referee_mock rqt_clean_start plain --list-plugins
```

## 已知问题与解决方案

### 1. `FakeLocation` 默认命名空间和行为树输入不一致

- 现状：`FakeLocation` 默认发布到 `/rm_referee/mock/*`
- 影响：`sentry_nav_bt_test` 默认只订阅 `/rm_referee/robot_pos`，所以 TF 以外的裁判系统位置链路不会自动接上
- 解决方案：联调 UC 树时把 `FakeLocation` 的话题前缀手动改成 `/rm_referee`

### 2. `MatchControl` 按规则执行 5 秒姿态冷却

- mock 接收并解析请求后，`Tx.response.ok` 表示链路处理成功；冷却中或强化额度耗尽时，姿态字段会在语义层被拒绝。
- 是否切换成功应以随后发布的 `/rm_referee/sentry_info` 为准。
- UI 的“强制设置”仅用于构造测试场景，可以绕过冷却，但不会补充已经耗尽的强化额度。

### 3. `/rm_referee/tx` 的 `response.ok` 只是链路层成功，不是语义层成功

- 现状：mock 只要能正确解析 `0x0301 / 0x0120`，通常就会返回 `ok=true`
- 影响：这个返回值不能直接等价成“姿态已切换成功”“复活已确认成功”或“能量机关已成功进入激活状态”
- 解决方案：
  - 姿态以 `/rm_referee/sentry_info` 里的 `current_posture` 为准
  - 打符以 `/rm_referee/event_data` 里的大小符状态为准
  - 复活以 `RobotStatus.current_hp`、金币和 `SentryInfo` 权限位为准

## Keyboard Publisher

![keyboard_publisher_rqt](../docs/keyboard_publisher_rqt.jpg)

模拟 2026 V1.3.0 自定义客户端的键鼠控制消息。Keyboard Publisher 会读取键盘输入，并将输入的数据封装成 `rm_referee_msgs/KeyboardMouseControl` 消息发布到指定话题上，默认话题为 `/rm_referee/mock/keyboard_mouse_control`。

> TODO: 鼠标位移和按键输入

## Dart Client

![dart_client_rqt](../docs/dart_client_rqt.jpg)

模拟比赛服务器下发的飞镖相关信息和云台手客户端的操作，包括：

- `0x0105` `rm_referee_msgs/DartInfo`：飞镖发射相关数据
- `0x020A` `rm_referee_msgs/DartClientCmd`：飞镖选手端指令数据

## Fake Location

![fake_location_rqt](../docs/fake_location_rqt.jpg)

模拟 UWB 定位数据。rqt 界面上每个小圆点代表一个机器人，拖动圆点可以改变机器人的位置，拖动圆点周围的小三角可以改变机器人的朝向。如果需要更精确的数值也可以直接输入位置和朝向数据。Fake Location 会根据以上数据构造发布 `rm_referee_msgs/RobotPos` 和 `rm_referee_msgs/GroundRobotPosition` 消息。另外，Fake Location 还支持给实际发布的假数据添加高斯噪声，以模拟真实环境下的定位误差。通过调整界面上的“位置噪声”参数可以控制噪声的大小。

默认话题前缀是 `/rm_referee/mock`。如果是给 `sentry_nav_bt_test` 喂数据，记得手动改成 `/rm_referee`。

> [!NOTE]  
> 根据裁判系统协议定义，只有“本机器人位置”(`rm_referee_msgs/RobotPos`)消息包含朝向信息。

> [!NOTE]
> `ground_pos_relay` 会按协议把哨兵收到的 `0x020B` 完整 40 字节封装进 `0x0301 / 0x0200` 机器人间通信，发送给同阵营雷达；当前 Match Control 的 `/rm_referee/tx` mock 仍只解析 `0x0301 / 0x0120` 哨兵自主决策指令。

> TODO: 通过 `0x0305` 小地图接收数据模拟雷达发送的敌方机器人位置数据

## Match Control [WIP!]

![match_control_rqt](../docs/match_control_rqt.jpg)

手动控制比赛数据发布。当前可以控制发布的数据包括：

- `0x0001` `rm_referee_msgs/GameStatus`：比赛状态
- `0x0003` `rm_referee_msgs/GameRobotHP`：己方机器人、双方建筑血量和伤害差
- `0x0101` `rm_referee_msgs/EventData`：场地事件数据
- `0x0201` `rm_referee_msgs/RobotStatus`：本机器人状态
- `0x0202` `rm_referee_msgs/PowerHeatData`：缓冲能量和射击热量
- `0x0206` `rm_referee_msgs/HurtData`：扣血信息
- `0x0208` `rm_referee_msgs/ProjectileAllowance`：允许发弹量和剩余金币
- `0x0209` `rm_referee_msgs/RFIDStatus`：RFID 模块状态
- `0x020D` `rm_referee_msgs/SentryInfo`：哨兵兑换、复活、姿态与能量机关激活状态

姿态规则模拟：

- 支持普通进攻/防御/移动和强化进攻/防御/移动共六种请求。
- 三种普通姿态各有独立的 180 秒弱化前累计时间；三种强化姿态各有独立的 15 秒累计时间。
- 强化姿态期间会同时消耗对应普通姿态时间，强化额度归零后自动回到对应普通姿态。
- UI 可直接编辑六个时间窗，并提供“强化进攻剩余 2s”“耗尽当前强化”等快速场景。
- “敌方前哨站归零”“模拟射击：热量 +10”和“连续受击 3 次”用于测试行为树结束条件、热量姿态选择和受击保持逻辑。

可以接收并回显的发送数据：

- 目前支持解析哨兵发送的 `0x0120` 子内容部分字段

补给区规则模拟：

- Match Control 会尝试读取 `sentry_nav_bt_test/config/waypoints.json` 里的 `supply_point` 和 `supply_point_2`，并通过 TF `map -> base_link` 或 `/rm_referee/robot_pos` 判断是否到达补给点。
- 到达补给点后发布 `RFIDStatus` 补给区 bit，`sentry_nav_bt_test` 会将其识别为 `rfid_supply_zone_detected=1`。
- 处于比赛中且占领补给区时，mock 按当前简化策略每秒恢复哨兵 `100 HP`，并按规则结算哨兵每分钟可领取的 `100` 发 17mm 允许发弹量；未领取的分钟数会累积，进补给区后一次性领取。
- 金币按规则低保模拟：初始至少 `400`，随后在 `05:59/04:59/03:59/02:59/01:59` 各加 `50`，`00:59` 加 `150`。

协议链路说明：

- 当前 Match Control 接收的是 `/rm_referee/tx` 已经成帧后的 ROS service 请求，并不模拟规则手册附录里的空口 15 字节分片、重复 Payload Length 校验或图传链路重复下发机制。
- 不建议在 `/rm_referee/tx` 层直接比较前后两包是否一致；哨兵姿态、买弹、复活确认本来就会连续发送不同内容，强行截断会误伤正常指令。
- mock 会按 `0x0120` 协议校验累计字段：补血点补弹累计值不能回退，远程补弹/回血请求次数不能回退且每次最多增加 1。不合法时会按协议只处理该字段前面的低位指令，并忽略该字段及其后面的高位指令。
- 如果需要测试“前后分片/重复包不一致时只接受问题字节前内容”这类抗干扰策略，建议新增一个原始字节流或图传链路 mock，并把校验逻辑放在字节流解析层，而不是放在当前语义层 service 回调里。

复活规则模拟：

- 哨兵 HP 变为 `0` 时开始复活读条，并将射击热量清零、缓冲能量恢复到 mock 的缓冲能量上限。
- 读条长度按 `10 + (420 - 比赛剩余时长) / 10 + 20 * 累计金币立即复活次数` 计算，小数四舍五入。
- 常规情况下每秒读条 `+1`；勾选补给区或基地血量低于 `2000` 时每秒 `+4`。
- 读条完成后置 `SentryInfo.can_confirm_resurrection`，收到 `0x0120 bit0` 后按上限血量 `10%` 回血。
- 战亡期间如果金币足够 `SentryInfo.buy_revive_cost`，置 `SentryInfo.can_buy_resurrection`，收到 `0x0120 bit1` 后扣金币并回满血，同时累计金币复活次数。
- 读条复活后 mock 会临时锁定发射机构；勾选补给区、基地增益点或前哨站增益点状态时视作检测到可占领交互模块卡并解除虚弱。

需要注意两点：

- `/rm_referee/tx` 的返回值更偏向“请求被 mock 正常解析”，不是最终状态确认
- 当前 mock 按规则内置姿态切换冷却和强化累计计时，更适合做裁判系统协议与策略联调，不等价于实车执行器响应时间

如果有需要，可以继续扩展更多比赛状态的控制项。

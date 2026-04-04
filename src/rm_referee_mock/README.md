# rm_referee_mock

这是 `rm_referee_ros2` 所属的一个子模块，提供了一些用于测试的 Mock 组件，用来模拟裁判系统的数据发送行为，方便在没有真实裁判系统或者不便搭建环境的情况下进行开发和测试。当前文档以《RoboMaster 2026 机甲大师高校系列赛通信协议 V1.3.0（20260327）》为准。

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

## 与 `sentry_nav_bt_test` / `uc.xml` 联调

如果目标是联调 `sentry_nav_bt_test` 当前默认的 `uc.xml`，建议按下面的方式使用：

1. `MatchControl`
   - 话题前缀保持默认 `/rm_referee`
   - `Robot ID` 设成 `7` 或 `107`
   - 比赛阶段切到 `[4] 比赛中`
   - 按需发布 `GameStatus`、`GameRobotHP`、`EventData`、`RobotStatus`、`ProjectileAllowance`、`SentryInfo`
2. `FakeLocation`
   - 必须把话题前缀从默认的 `/rm_referee/mock` 改成 `/rm_referee`
   - 否则 `sentry_nav_bt_test` 收不到 `/rm_referee/robot_pos`
3. 如果要验证打符流程
   - 主要观察 `/rm_referee/event_data` 里的大小符状态
   - `Tx.srv` 的 `ok` 更接近“包已解析”而不是“行为一定生效”

## 已知问题与解决方案

### 1. `FakeLocation` 默认命名空间和行为树输入不一致

- 现状：`FakeLocation` 默认发布到 `/rm_referee/mock/*`
- 影响：`sentry_nav_bt_test` 默认只订阅 `/rm_referee/robot_pos`，所以 TF 以外的裁判系统位置链路不会自动接上
- 解决方案：联调 UC 树时把 `FakeLocation` 的话题前缀手动改成 `/rm_referee`

### 2. `MatchControl` 的姿态冷却默认是 5 秒，不适合直接压测 UC 树

- 现状：mock 在处理 `/rm_referee/tx` 的姿态切换时，默认加了 `5.0 s` 冷却
- 影响：`uc.xml` 里很多 `SetSentryPosture` 节点超时只有 `300 ms`，mock 会把本来正常的姿态切换请求表现成“超时失败”
- 解决方案：
  - 用 mock 联调 UC 时，不要直接拿“姿态切换失败”判断策略错了
  - 先单独验证 `/rm_referee/tx -> /rm_referee/sentry_info.current_posture` 闭环延迟
  - 如果后面要做严格联调，建议让 mock 冷却时间和 BT 超时参数对齐

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

> TODO: 通过 0x0301 多机通信消息模拟雷达发送的敌方机器人位置数据

## Match Control [WIP!]

![match_control_rqt](../docs/match_control_rqt.jpg)

手动控制比赛数据发布。当前可以控制发布的数据包括：

- `0x0001` `rm_referee_msgs/GameStatus`：比赛状态
- `0x0003` `rm_referee_msgs/GameRobotHP`：己方机器人血量
- `0x0101` `rm_referee_msgs/EventData`：场地事件数据
- `0x0201` `rm_referee_msgs/RobotStatus`：本机器人状态
- `0x0206` `rm_referee_msgs/HurtData`：扣血信息
- `0x0208` `rm_referee_msgs/ProjectileAllowance`：允许发弹量和剩余金币
- `0x020D` `rm_referee_msgs/SentryInfo`：哨兵兑换、复活、姿态与能量机关激活状态

可以接收并回显的发送数据：

- 目前支持解析哨兵发送的 `0x0120` 子内容部分字段

需要注意两点：

- `/rm_referee/tx` 的返回值更偏向“请求被 mock 正常解析”，不是最终状态确认
- 当前 mock 内置了姿态切换冷却，更适合做协议联调，不适合直接当作“实车姿态执行器”的等价替身

如果有需要，可以继续扩展更多比赛状态的控制项。

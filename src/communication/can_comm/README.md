# can_comm

NUC ↔ MCU 的 CAN 总线通信包，是 `serial_comm`（USB 串口）的替代方案。负责将 Nav2 速度指令和感知信息下发 MCU，同时接收 MCU 上报的目标检测数据（yaw/pitch/distance）。基于 [librm]进行 CAN 硬件访问。

## 组件概览

| 组件 | 可执行文件 | 方向 | 功能 |
|------|-----------|------|------|
| `can_comm_node` | `can_comm_node` | 发送 | NUC → MCU：下发运动指令、扫描模式、装甲板检测 |
| `can_receive_node` | `can_receive_node` | 接收 | MCU → NUC：接收目标 yaw/pitch/distance、电控 yaw |
| `target_frame_node` | `target_frame_node` | 处理 | 将 CAN 接收的目标球坐标转为 map 系 PoseStamped |
| `yaw_controller_node` | `yaw_controller_node` | 处理 | NLJG/前哨站定向 yaw 控制器（零飘补偿） |
| `can_debug_gui` | `can_debug_gui.py` | 调试 | tkinter GUI 面板，监控 CAN 收发数据 |

## 数据流全景

```
                      ┌──────────────────────────┐
                      │          MCU             │
                      │   (STM32 底层控制器)       │
                      └──────┬─────────┬─────────┘
                             │ CAN TX  │ CAN RX
                      ┌──────▼──┐  ┌──┴──────────┐
                      │ 0x180   │  │   0x190      │
                      │ 接收帧   │  │   发送帧      │
                      └──────┬──┘  └──┬──────────┘
                             │        │
              ┌──────────────▼──┐  ┌──▼──────────────────────┐
              │ can_receive_node│  │ can_comm_node (发送)      │
              │   (SocketCAN)   │  │     (librm)              │
              │                 │  │                          │
              │ 发布:           │  │ 订阅:                     │
              │  /target/yaw ◄──┼──┤  /cmd_vel                │
              │  /target/pitch  │  │  /vw                     │
              │  /target/distance│ │  /scan_mod_type          │
              │  /diankong/yaw  │  │  /auto_shoot_type        │
              └──────┬──────────┘  │  /outpost_mode_type      │
                     │             │  /detector/armor_presence│
       ┌─────────────┼───────┐     │  /target_yaw ◄── 指令yaw │
       │             │       │     │  /rm_referee/game_status │
       ▼             ▼       ▼     └─────────────────────────┘
  /target/yaw  /target/  /target/            ▲
  (感知yaw)    pitch    distance             │
       │             │       │               │
       └──────┬──────┘       │    yaw_controller_node
              ▼              │     │ ← /yaw_controller (0=NLJG, 1=outpost)
     target_frame_node       │     │ ← /diankong/yaw (零飘补偿)
       │                     │     │
       ▼                     │     ▼
  /detected_target_pose      │   发布 /target_yaw (指令yaw, float32度)
  (ElasticTracker 订阅)       │
                             │
  ┌──────────────────────────┘
```

## 各组件详解

### can_comm_node — CAN 发送

NUC → MCU 运动控制帧（CAN ID `0x190`，500Hz，总帧长 6 字节）：

```
 Byte 0        Byte 1        Byte 2        Byte 3        Byte 4                     Byte 5
┌────────────┬────────────┬────────────┬────────────┬─────────────────────────┬────────────┐
│    vx      │    vy      │ target_yaw │ target_yaw │    模式/装甲板标志位      │    vw      │
│   int8     │   int8     │  (高字节)   │  (低字节)   │     bit flags           │   int8     │
│            │            │            │            │                         │            │
│  v ∈ [-1,1]│ v ∈ [-1,1] │  int16 BE  │  int16 BE  │ bit[0] left             │ v ∈ [-1,1] │
│  × 127     │  × 127     │   × 100    │   × 100    │ bit[1] behind           │  × 127     │
│            │            │  单位: 度   │  单位: 度   │ bit[2] right            │            │
│            │            │            │            │ bit[3] (保留=0)         │            │
│  1 Byte    │  1 Byte    │  1 Byte    │  1 Byte    │ bit[4] scan_mode        │  1 Byte    │
│            │            │            │            │ bit[5] NLJG_mode        │            │
│            │            │            │            │ bit[6] outpost_mode     │            │
│            │            │            │            │ bit[7] (保留=0)         │            │
└────────────┴────────────┴────────────┴────────────┴─────────────────────────┴────────────┘
```

**Byte 0 — vx (int8)**：
- 来源：`/cmd_vel` (Twist.linear.x)，float m/s
- 编码：clamp(vx, -1.0, 1.0) × 127 → int8
- 位表示：8-bit 有符号整数，`0x7F` = +1.0 m/s，`0x81` = -1.0 m/s，`0x00` = 0

**Byte 1 — vy (int8)**：
- 来源：`/cmd_vel` (Twist.linear.y)，float m/s
- 编码：clamp(vy, -1.0, 1.0) × 127 → int8

**Byte 2..3 — target_yaw (int16, big-endian, ×100)**：
- 来源：`/target_yaw` (Float32)，**大 yaw 指令角度**，单位度
- 编码：clamp(yaw, -180.0, 180.0) × 100 → int16 big-endian
- Byte 2: `(val >> 8) & 0xFF`（高字节），Byte 3: `val & 0xFF`（低字节）
- 示例：90.00° → 9000 → `0x23 0x28`；-90.00° → -9000 → `0xDC 0xD8`

**Byte 4 — 标志位 (bit flags)**：

| Bit | 名称 | 来源话题 | 含义 |
|-----|------|----------|------|
| 0 | left | `/detector/armor_presence` (ArmorPresence.left) | 左侧装甲板检测到目标 |
| 1 | behind | `/detector/armor_presence` (ArmorPresence.behind) | 后方装甲板检测到目标 |
| 2 | right | `/detector/armor_presence` (ArmorPresence.right) | 右侧装甲板检测到目标 |
| 3 | — | — | 保留，固定为 0 |
| 4 | scan_mode | `/scan_mod_type` (ScanMode.scan_mod_type) | 扫描模式使能 |
| 5 | NLJG_mode | `/auto_shoot_type` (Bool.data) | 能量机关 (NLJG) 模式 |
| 6 | outpost_mode | `/outpost_mode_type` (Bool.data) | 前哨站模式 |
| 7 | — | — | 保留，固定为 0 |

**Byte 5 — vw (int8)**：
- 来源：`/vw` (Vw.vw)，float rad/s
- 编码：clamp(vw, -1.0, 1.0) × 127 → int8
- 用途：MPPI 上坡阶段的云台 Vw 持续输出

**话题订阅**：

| 话题 | 类型 | 说明 |
|------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 速度指令 (vx, vy) |
| `/vw` | `sentry_msgs/Vw` | MPPI 上坡 Vw |
| `/target_yaw` | `std_msgs/Float32` | 目标 yaw 角度 |
| `/scan_mod_type` | `sentry_msgs/ScanMode` | 扫描模式 |
| `/auto_shoot_type` | `std_msgs/Bool` | NLJG 模式（能量机关） |
| `/outpost_mode_type` | `std_msgs/Bool` | 前哨站模式 |
| `/detector/armor_presence` | `sentry_msgs/ArmorPresence` | 装甲板检测 (left/behind/right) |
| `/rm_referee/game_status` | `rm_referee_msgs/GameStatus` | 裁判系统比赛状态 |

**超时安全**：`cmd_vel` 和 `vw` 均设超时门控，超时后自动归零。

### can_receive_node — CAN 接收

MCU → NUC 目标检测帧（CAN ID `0x180`，SocketCAN 原生接口，总帧长 8 字节）：

```
 Byte 0        Byte 1        Byte 2        Byte 3        Byte 4        Byte 5        Byte 6         Byte 7
┌────────────┬────────────┬────────────┬────────────┬────────────┬────────────┬─────────────┬──────────────┐
│  小 yaw     │  小 yaw     │   pitch    │   pitch    │  distance  │  distance  │ diankong_yaw│ diankong_yaw │
│  (高字节)   │  (低字节)   │  (高字节)   │  (低字节)   │  (高字节)   │  (低字节)   │  (高字节)    │  (低字节)     │
│            │            │            │            │            │            │             │              │
│  int16 BE  │  int16 BE  │  int16 BE  │  int16 BE  │  int16 BE  │  int16 BE  │  int16 BE   │  int16 BE    │
│   × 100    │   × 100    │   × 100    │   × 100    │   × 100    │   × 100    │   × 100     │   × 100      │
│  单位: 度   │  单位: 度   │  单位: 度   │  单位: 度   │  单位: 米   │  单位: 米   │  单位: 度    │  单位: 度     │
└────────────┴────────────┴────────────┴────────────┴────────────┴────────────┴─────────────┴──────────────┘
```

**Byte 0..1 — 小 yaw (int16, big-endian, ×100)**：
- 话题：发布到 `/target/yaw` (Float32)
- 含义：视觉检测到的目标方位角，**小 yaw**，云台精瞄用
- 朝向：MCU 上报 0° = base_link 后方；`target_frame_node` 消费时做 +180° 翻转为 0° = base_link 前方
- 解码：`yaw = int16_BE(data[0:2]) / 100.0`，单位度
- 示例：`0x23 0x28`（大端 9000）→ 90.00°；`0xDC 0xD8`（大端 -9000）→ -90.00°

**Byte 2..3 — pitch (int16, big-endian, ×100)**：
- 话题：发布到 `/target/pitch` (Float32)
- 含义：视觉检测到的目标俯仰角
- 朝向：0° = 水平，正值 = 仰角向上
- 解码：`pitch = int16_BE(data[2:4]) / 100.0`，单位度

**Byte 4..5 — distance (int16, big-endian, ×100)**：
- 话题：发布到 `/target/distance` (Float32)
- 含义：视觉检测到的目标距离（斜距）
- 解码：`distance = int16_BE(data[4:6]) / 100.0`，单位米
- 特殊值：distance ≤ 0 视为无效目标，`target_frame_node` 不发布 TF 和 Pose

**Byte 6..7 — diankong_yaw (int16, big-endian, ×100)**：
- 话题：发布到 `/diankong/yaw` (Float32)
- 含义：电控实时反馈的当前 yaw 角，**大 yaw**，用于 `yaw_controller` 的零飘补偿
- 朝向：由 MCU 定义（与电控 IMU/编码器坐标系一致）
- 解码：`diankong_yaw = int16_BE(data[6:8]) / 100.0`，单位度

**话题发布**：

| 话题 | 类型 | yaw 类型 | 说明 |
|------|------|----------|------|
| `/target/yaw` | `std_msgs/Float32` | **小 yaw** | 视觉检测目标方位角 (度) |
| `/target/pitch` | `std_msgs/Float32` | — | 目标 pitch 角 (度) |
| `/target/distance` | `std_msgs/Float32` | — | 目标距离 (m) |
| `/diankong/yaw` | `std_msgs/Float32` | **大 yaw** | 电控实时 yaw (度) |

> [!NOTE]
> **2026 赛季哨兵为大小 yaw 双轴构型**：大 yaw 负责底盘/粗指向，小 yaw 负责云台精瞄。`/target/yaw` 是小 yaw 角度；`/target_yaw`（指令）和 `/diankong/yaw` 是大 yaw 角度。

### target_frame_node — 目标坐标转换

将 CAN 接收的球坐标 (yaw, pitch, distance) 转换为笛卡尔坐标：

1. **yaw 校正**：MCU 上报 0°=base_link 后方 → +180° 翻转到 base_link 前向基准
2. **球→直角**：`x = distance×cos(pitch)×cos(yaw)`, `y = distance×cos(pitch)×sin(yaw)`
3. **发布 TF**：`base_link → target_frame`
4. **TF 转换**：变换到 map 系，发布 `/detected_target_pose` (PoseStamped)
5. **自动导航**：当 planner 切换到 `ElasticTracker` 时，自动发送 `NavigateToPose` action 触发追踪

### yaw_controller_node — 定向 yaw 控制器

计算机器人到固定目标（NLJG 能量机关 / 前哨站）的 yaw 角，持续发布给 can_send 下发 MCU：

- **触发**：订阅 `/yaw_controller` (`Int32`，0=NLJG, 1=outpost)
- **锁定**：首次触发时计算当前 base_link → 目标点的角度并锁定
- **零飘补偿**：每帧计算 `offset = diankong_yaw - TF_yaw`，将 offset 叠加到目标 yaw
- **连续发布**：补偿后的 yaw 持续发布到 `/target_yaw`

## 角度与 yaw 类型约定

2026 赛季哨兵采用**大小 yaw 双轴构型**：大 yaw 负责底盘粗指向，小 yaw 负责云台精瞄。

| yaw 类型 | 话题 | 方向 | 含义 |
|----------|------|------|------|
| **小 yaw** | `/target/yaw` | MCU → NUC | 视觉检测到的目标方位角，是小云台角度 |
| **大 yaw** | `/target_yaw` | NUC → MCU | 指令 yaw，告诉电控大yaw指向哪个方向 |
| **大 yaw** | `/diankong/yaw` | MCU → NUC | 电控实时反馈的当前 大yaw 角，用于零飘补偿 |

### 角度朝向约定

所有角度单位为**度 (°)**，俯视逆时针为正（右手定则）。

| 话题 | 0° 方向 | 取值范围 | 编码 (CAN) |
|------|---------|----------|------------|
| `/target/yaw`（小 yaw） | MCU 上报 0° = base_link 后方；经 `target_frame_node` 内部 +180° 翻转后 0° = base_link 前方 | `clampAngleDeg` 归一化到 `[-180, 180]` | int16 ×100 |
| `/target/yaw`（指令，大 yaw） | map 系下 atan2(dy, dx)，0° = map X 轴正向 | 归一化到 `(-180, 180]`，CAN 发送时限幅 `[-180, 180]` | int16 ×100, big-endian |
| `/diankong/yaw`（大 yaw） | 由 MCU 定义（与电控 IMU/编码器一致） | 取决于 MCU，解码不做限幅 | int16 ×100 |
| `/target/pitch` | 0° = 水平，正值 = 仰角向上 | 取决于 MCU 视觉检测范围 | int16 ×100 |

### `/target/yaw` 的方向翻转

MCU 上报的小 yaw 以 base_link **后方**为 0°（即机器人尾部方向）。`target_frame_node` 在使用前做 +180° 翻转：

```cpp
// yaw: 输入0°=base_link后方 → 加180°换到base_link前向基准，右手定则
const double corrected_yaw_deg = clampAngleDeg(yaw_deg + 180.0);
```

翻转后 0° = base_link 前方，符合导航系统中 base_link 的常规朝向约定。

## 话题总览

### 订阅

| 话题 | 消费者 |
|------|--------|
| `/cmd_vel` | can_comm_node |
| `/vw` | can_comm_node |
| `/scan_mod_type` | can_comm_node |
| `/auto_shoot_type` | can_comm_node |
| `/outpost_mode_type` | can_comm_node |
| `/detector/armor_presence` | can_comm_node |
| `/rm_referee/game_status` | can_comm_node |
| `/target_yaw` | can_comm_node |
| `/target/yaw` | target_frame_node |
| `/target/pitch` | target_frame_node |
| `/target/distance` | target_frame_node |
| `/diankong/yaw` | yaw_controller_node |
| `/yaw_controller` | yaw_controller_node |
| `/planner_name` | target_frame_node |

### 发布

| 话题 | 发布者 |
|------|--------|
| `/target/yaw` | can_receive_node |
| `/target/pitch` | can_receive_node |
| `/target/distance` | can_receive_node |
| `/diankong/yaw` | can_receive_node |
| `/detected_target_pose` | target_frame_node |
| `/target_yaw` | yaw_controller_node |

## 启动

```bash
# 完整启动（4 个节点）
ros2 launch can_comm can_comm.launch.py

# 调试启动（GUI + can_send + can_receive）
ros2 launch can_comm debug_can.launch.py
```

## 编译

```bash
colcon build --packages-select can_comm
```

## 配置文件

所有参数由 `bringup` 包统一管理：[`bringup/config/can_params.yaml`](../../../bringup/config/can_params.yaml)

## 文件结构

```
can_comm/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── can_params.yaml              # 所有节点参数（bringup 管理）
├── launch/
│   ├── can_comm.launch.py           # 完整启动
│   └── debug_can.launch.py          # 调试 GUI 启动
├── librm/                           # RoboMaster CAN 硬件库 (git submodule)
├── scripts/
│   └── can_debug_gui.py             # tkinter 调试面板
└── src/
    ├── can_send.cpp                 # CAN 发送节点（librm）
    ├── can_receive.cpp              # CAN 接收节点（SocketCAN）
    ├── target_frame_node.cpp        # 目标坐标转换 + 自动导航
    └── yaw_controller_node.cpp  # NLJG/前哨站 yaw 控制器
```

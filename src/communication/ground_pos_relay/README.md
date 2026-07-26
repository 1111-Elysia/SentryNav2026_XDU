# ground_pos_relay

将裁判系统下发的己方地面机器人位置数据（`0x020B`）通过机器人间通信（`0x0301 / 0x0200`）中继给同阵营雷达站。同时提供坐标系转换、GUI 模拟位置等配套工具，用于开发、测试和联调。

该包基于 RoboMaster 2026 裁判系统通信协议，按规则手册附录实现完整的帧封装（CRC8 头校验 + CRC16 尾校验）。

## 组件概览

| 组件 | 语言 | 功能 |
|------|------|------|
| `ground_pos_relay_node` | C++ | 核心中继：监听裁判系统 `0x020B`，封装 `0x0301/0x0200` 帧并通过 `/rm_referee/tx` 发送 |
| `ground_pos_relay_sim_node` | C++ | 模拟中继：监听 GUI 模拟器的位置数据，封装并发送（不依赖 TF） |
| `teammate_frame_converter_node` | C++ | 坐标转换：将官方地图系中的队友坐标转换到 odom 系 |
| `ground_pos_simulator.py` | Python | tkinter GUI：手动输入并发布模拟的地面机器人位置 |

## 通信链路

```
裁判系统 (0x020B)  ──→  /rm_referee/ground_robot_position
                              │
                              ▼
                   ground_pos_relay_node
                     (封装 0x0301/0x0200)
                              │
                              ▼
                      /rm_referee/tx  ──→  同阵营雷达站 (receiver_id=9/109)
```

### 发送/接收 ID 对照

| 本机 robot_id | 阵营 | sender_id | receiver_id |
|---------------|------|-----------|-------------|
| 7 | 红方哨兵 | 7 | 9 |
| 107 | 蓝方哨兵 | 107 | 109 |

## 坐标系转换

`ground_pos_relay_node` 和 `teammate_frame_converter` 共用相同的坐标变换参数，互为逆运算。

### 官方地图系 → odom 系（teammate_frame_converter）

```
odom_x =  cos(yaw) * (official_x - origin_x) + sin(yaw) * (official_y - origin_y)
odom_y = -sin(yaw) * (official_x - origin_x) + cos(yaw) * (official_y - origin_y)
```

### odom 系 → 官方地图系（ground_pos_relay_node 中 `use_self_position`）

```
official_x = origin_x + cos(yaw) * odom_x - sin(yaw) * odom_y
official_y = origin_y + sin(yaw) * odom_x + cos(yaw) * odom_y
```

其中红/蓝双方各有独立的原点 `(origin_x, origin_y)` 和朝向 `origin_yaw`，通过配置文件指定。

## 话题 & 服务

### 订阅

| 话题 | 节点 | 说明 |
|------|------|------|
| `/rm_referee/ground_robot_position` | relay, converter | 裁判系统下发的己方机器人位置 (0x020B) |
| `/rm_referee/robot_status` | relay, converter | 本机器人状态，用于获取 robot_id |
| `/ground_pos_sim/ground_robot_position` | sim_node | GUI 模拟器发布的模拟位置 |

### 发布

| 话题 | 节点 | 说明 |
|------|------|------|
| `/ground_pos_relay/teammate_pos_odom` | converter | 转换到 odom 系的队友坐标 |
| `/ground_pos_sim/ground_robot_position` | simulator | GUI 模拟位置数据 |

### 服务客户端

| 服务 | 节点 | 说明 |
|------|------|------|
| `/rm_referee/tx` | relay, sim_node | 发送封装好的裁判系统帧 |

## 编译

```bash
# 在 ROS 2 工作空间中
colcon build --packages-select ground_pos_relay
```

依赖：
- `rclcpp` / `rclpy`
- `rm_referee_msgs`
- `tf2` / `tf2_ros`

## 启动

### 真实模式（使用裁判系统数据 + TF 坐标转换）

```bash
ros2 launch ground_pos_relay ground_pos_relay.launch.py
```

启动 `ground_pos_relay_node` 和 `teammate_frame_converter_node`，从配置文件加载红蓝双方原点参数。

### 模拟模式（使用 GUI 手动输入位置）

```bash
ros2 launch ground_pos_relay ground_pos_relay_sim.launch.py
```

启动 Python GUI 模拟器和模拟中继节点：

![ground_pos_simulator 界面示意](ground_pos_simulator.png)

GUI 中可分别设置英雄、工程、步兵3号、步兵4号的 X/Y 坐标（官方地图系），支持：
- **发送一次**：手动触发单次发布
- **开始自动发送**：按设定频率周期性发布（1 ~ 50 Hz）
- `reserved` / `reserved_2` 字段代表哨兵位置

### 单节点启动

```bash
# 核心中继节点
ros2 run ground_pos_relay ground_pos_relay_node --ros-args --params-file config/ground_pos_relay.yaml

# 坐标转换节点
ros2 run ground_pos_relay teammate_frame_converter_node --ros-args --params-file config/ground_pos_relay.yaml

# 模拟中继节点
ros2 run ground_pos_relay ground_pos_relay_sim_node --ros-args -p robot_id:=7

# GUI 模拟器
ros2 run ground_pos_relay ground_pos_simulator.py
```

## 配置参数

配置文件：[config/ground_pos_relay.yaml](config/ground_pos_relay.yaml)

### ground_pos_relay_node

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `use_self_position` | bool | true | 是否通过 TF 将哨兵自身 odom 坐标换算为官方坐标填入 reserved 字段 |
| `odom_frame_id_` | string | "odom" | odom 坐标系 frame_id |
| `robot_frame_id_` | string | "base_link" | 机器人 base_link frame_id |
| `red.red_origin_x` | float | 0.0 | 红方地图原点 X（官方坐标系） |
| `red.red_origin_y` | float | 0.0 | 红方地图原点 Y（官方坐标系） |
| `red.red_origin_yaw` | float | 0.0 | 红方地图原点朝向（弧度） |
| `blue.blue_origin_x` | float | 0.0 | 蓝方地图原点 X（官方坐标系） |
| `blue.blue_origin_y` | float | 0.0 | 蓝方地图原点 Y（官方坐标系） |
| `blue.blue_origin_yaw` | float | 0.0 | 蓝方地图原点朝向（弧度） |

> [!NOTE]
> 红/蓝双方原点参数用于将 SLAM 里程计坐标转换为裁判系统 28 m × 15 m 官方场地坐标。典型值如红方原点 `(3.725, 7.5)`、蓝方原点 `(24.275, 7.5)`，红方 yaw = `0.0`，蓝方 yaw = `π`（朝向场地中心）。

### ground_pos_relay_sim_node

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `robot_id` | int | 7 | 模拟的哨兵 ID，7=红方，107=蓝方 |

### teammate_frame_converter_node

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `odom_frame_id` | string | "odom" | 输出坐标的 frame_id |
| `red_origin_x` | float | 0.0 | 红方地图原点 X |
| `red_origin_y` | float | 0.0 | 红方地图原点 Y |
| `red_origin_yaw` | float | 0.0 | 红方地图原点朝向（弧度） |
| `blue_origin_x` | float | 0.0 | 蓝方地图原点 X |
| `blue_origin_y` | float | 0.0 | 蓝方地图原点 Y |
| `blue_origin_yaw` | float | 0.0 | 蓝方地图原点朝向（弧度） |

## 协议帧结构

`ground_pos_relay_node` 和 `ground_pos_relay_sim_node` 构造的完整帧共 **55 字节**：

| 偏移 | 长度 | 字段 | 说明 |
|------|------|------|------|
| 0 | 1 | SOF | 帧头 `0xA5` |
| 1 | 2 | DataLen | 数据段长度 = 46（小端） |
| 3 | 1 | Seq | 帧序号（自增） |
| 4 | 1 | CRC8 | Header 校验（byte 0..3） |
| 5 | 2 | CmdID | `0x0301`（小端） |
| 7 | 2 | SubContentID | `0x0200`（小端） |
| 9 | 2 | SenderID | 7 或 107 |
| 11 | 2 | ReceiverID | 9 或 109 |
| 13 | 40 | Payload | 10 × float32 LE（英雄/工程/步兵3/步兵4/哨兵 各 x,y） |
| 53 | 2 | CRC16 | 整帧校验 |

Payload 中 10 个 float32 的顺序：

| 索引 | 字段 | 说明 |
|------|------|------|
| 0 | hero_x | 英雄 X |
| 1 | hero_y | 英雄 Y |
| 2 | engineer_x | 工程 X |
| 3 | engineer_y | 工程 Y |
| 4 | standard_3_x | 步兵 3 号 X |
| 5 | standard_3_y | 步兵 3 号 Y |
| 6 | standard_4_x | 步兵 4 号 X |
| 7 | standard_4_y | 步兵 4 号 Y |
| 8 | sentry_x | 哨兵 X（原 `reserved`） |
| 9 | sentry_y | 哨兵 Y（原 `reserved_2`） |

> [!NOTE]
> 当 `ground_pos_relay_node` 的 `use_self_position` 为 true 时，哨兵坐标会从 TF (`odom → base_link`) + 阵营原点 计算得到官方坐标系下的位置，覆盖原 `reserved` / `reserved_2` 字段。

## 文件结构

```
ground_pos_relay/
├── CMakeLists.txt                      # ament_cmake 构建脚本
├── package.xml
├── config/
│   └── ground_pos_relay.yaml           # 节点参数配置
├── launch/
│   ├── ground_pos_relay.launch.py      # 真实模式 launch
│   └── ground_pos_relay_sim.launch.py  # 模拟模式 launch
├── scripts/
│   └── ground_pos_simulator.py         # tkinter GUI 模拟器
└── src/
    ├── ground_pos_relay_node.cpp       # 核心中继节点
    ├── ground_pos_relay_sim_node.cpp   # 模拟中继节点
    └── teammate_frame_converter.cpp    # 坐标转换节点
```

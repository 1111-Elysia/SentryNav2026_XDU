# serial_comm

> [!WARNING]  
> **已废弃**。该包曾是 NUC → 底层 MCU 的 USB 串口通信链路，现已被其他通信方式替代。保留此代码仅供历史参考。

## 功能

将 Nav2 输出的速度指令（`/cmd_vel`）、自适应 MPPI 的 Vw 信号（`/vw`）和扫描模式（`/scan_mod_type`）打包为二进制帧，通过 USB 串口发送给底层 MCU（如 STM32），由 MCU 执行电机控制和云台扫描。

## 协议帧格式

```
┌──────┬──────┬─────────┬─────────┬───────────┬─────────┬───────────────┬──────┐
│ SOF  │  ID  │   vx    │   vy    │   vyaw    │   vw    │ scan_mod_type │ EOF  │
│ 0x55 │ 0x04 │ float32 │ float32 │  float32  │ float32 │     bool      │ 0xFF │
│  1B  │  1B  │   4B    │   4B    │    4B     │   4B    │      1B       │  1B  │
└──────┴──────┴─────────┴─────────┴───────────┴─────────┴───────────────┴──────┘
                                     总帧长 = 20 字节
```

| 字段 | 类型 | 来源话题 | 说明 |
|------|------|----------|------|
| SOF | uint8 (0x55) | — | 帧头标识 |
| ID | uint8 (0x04) | — | 帧类型 / 设备 ID |
| vx | float32 | `/cmd_vel` (linear.x) | X 轴线速度 (m/s) |
| vy | float32 | `/cmd_vel` (linear.y) | Y 轴线速度 (m/s) |
| vyaw | float32 | `/cmd_vel` (angular.z) | 角速度 — **当前代码中强制置 0** |
| vw | float32 | `/vw` | 云台 Vw 角速度 (rad/s，上坡卡住检测) |
| scan_mod_type | bool | `/scan_mod_type` | 扫描模式切换 |
| EOF | uint8 (0xFF) | — | 帧尾标识 |

## 话题

| 话题 | 方向 | 类型 | 说明 |
|------|------|------|------|
| `/cmd_vel` | 订阅 | `geometry_msgs/Twist` | Nav2 速度指令 |
| `/vw` | 订阅 | `sentry_msgs/Vw` | MPPI 上坡 Vw 输出 |
| `/scan_mod_type` | 订阅 | `sentry_msgs/ScanMode` | 扫描模式切换信号 |

## 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `port` | string | `/dev/ttyACM0` | 串口设备路径 |
| `baudrate` | int | 115200 | 波特率 |
| `send_frequency` | int | 500 | 发送频率 (Hz) |
| `cmd_vel_topic` | string | `/cmd_vel` | 速度指令话题 |
| `vw_topic` | string | `/vw` | Vw 信号话题 |
| `scan_mod_type_topic` | string | `/scan_mod_type` | 扫描模式话题 |

配置文件由 `bringup` 包管理：[`bringup/config/serial_params.yaml`](../../../bringup/config/serial_params.yaml) 和 [`bringup/config/topic_names.yaml`](../../../bringup/config/topic_names.yaml)。

## 使用（已废弃，仅供参考）

```bash
ros2 launch serial_comm serial_comm.launch.py
```

注意：标记了 `COLCON_IGNORE`，不参与全量 colcon 构建。如需编译：

```bash
colcon build --packages-select serial_comm
```

## 已知问题 / 遗留

- **`vyaw` 强制置 0**：在 `cmdVelCallback` 中将 `angular.z` 赋值后立即覆盖为 0，意味着角速度从未实际下发。原计划由 MCU 自行计算全向轮分解
- **单向通信**：仅 NUC → MCU 发送，无接收。配置文件中有 `serial_receive_node` 的预留参数，但从未实现
- **Boost.Asio 实现**：`include/serial_comm/serial_port.hpp` 中的 `termios` 版 `SerialPort` 类未被实际使用，实际采用 Boost.Asio
- **match_stage_**：代码中声明了 `match_stage_` 成员变量并用于日志输出，但从未被赋值（始终为 0），推测为比赛阶段联动功能的一部分

## 文件结构

```
serial_comm/
├── CMakeLists.txt
├── COLCON_IGNORE                       # 不参与全量构建
├── package.xml
├── config/
│   ├── serial_params.yaml              # 串口参数 + 预留接收节点参数
│   └── topic_names.yaml                # 话题名映射
├── include/serial_comm/
│   └── serial_port.hpp                 # termios SerialPort 类（未使用）
├── launch/
│   └── serial_comm.launch.py           # 启动文件（引用 bringup 包配置）
└── src/
    └── serial_comm_node.cpp            # 主节点实现（Boost.Asio）
```

# teammate_cost_layer

Nav2 costmap 插件层。订阅己方队友机器人位置（odom 系），在每个队友周围创建**圆形渐变代价区域**，让 MPPI 控制器通过梯度感知自然绕开友方单位，避免碰撞。

配合 `ground_pos_relay` 包的 `teammate_frame_converter` 节点使用，构成完整的"裁判系统位置 → 坐标转换 → costmap 避让"管线。

## 代价模型

```
                     总半径 = core_radius + decay_radius
                                    
                   ┌─────────────────────────────┐
                   │         衰减区               │
                   │  cost ∝ (1 - t)^power       │
                   │  t = (d - core) / decay      │
                   │     ┌───────────────┐        │
                   │     │   核心致命区    │        │
                   │     │  cost = 254   │        │
                   │     │   (固定)       │        │
                   │     └───────────────┘        │
                   │          队友位置             │
                   └─────────────────────────────┘
```

- **核心区** (`d ≤ core_radius`)：固定代价值 (默认 254 = LETHAL)，绝对不可进入
- **衰减区** (`core < d ≤ core+decay`)：幂梯度衰减，`cost = peak × (1 - t)^power`
- **区外**：无代价

MPPI 的 sampled trajectory 机制能感知衰减区的代价梯度，平滑绕开队友，避免 A* 规划器的硬障碍带来的突然绕行。

## 数据流

```
裁判系统 (0x020B)
     │
     ▼
ground_pos_relay / teammate_frame_converter
     │  (官方地图系 → odom 系转换)
     ▼
/ground_pos_relay/teammate_pos_odom  (GroundRobotPosition)
     │
     ▼
teammate_cost_layer
     │  (圆形渐变代价)
     ▼
costmap → MPPI 梯度避障
```

## 插件信息

| 属性 | 值 |
|------|-----|
| 插件类 | `teammate_cost_layer::TeammateCostLayer` |
| 基类 | `nav2_costmap_2d::Layer` |
| 插件名（yaml 配置） | `teammate_cost_layer::TeammateCostLayer` |

## 参数

### 基本参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `enabled` | bool | true | 是否启用 |
| `teammate_topic` | string | `/ground_pos_relay/teammate_pos_odom` | 队友位置订阅话题 |
| `teammate_timeout` | float | 2.0 | 队友数据超时 (s)，超时后不施加代价 |

### 队友选择

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `enable_hero` | bool | true | 对英雄添加代价 |
| `enable_engineer` | bool | true | 对工程添加代价 |
| `enable_standard_3` | bool | false | 对步兵 3 添加代价 |
| `enable_standard_4` | bool | false | 对步兵 4 添加代价 |
| `enable_sentry` | bool | true | 对（对方）哨兵添加代价 |

### 代价区域

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `core_radius` | float | 0.5 | 核心致命区半径 (m) |
| `decay_radius` | float | 0.5 | 衰减区宽度 (m)，总半径 = core + decay |
| `core_cost_value` | int | 254 | 核心区代价值 (0–254) |
| `gradient_power` | float | 1.5 | 衰减指数 (1=线性, 2=二次, 0.5=凹) |

## 在 costmap 中使用

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["static_layer", "teammate_layer", "inflation_layer"]

      teammate_layer:
        plugin: "teammate_cost_layer::TeammateCostLayer"
        enabled: true
        teammate_topic: "/ground_pos_relay/teammate_pos_odom"
        teammate_timeout: 2.0

        enable_hero: true
        enable_engineer: true
        enable_standard_3: false
        enable_standard_4: false
        enable_sentry: true

        core_radius: 0.5
        decay_radius: 0.5
        core_cost_value: 254
        gradient_power: 1.5
```

> [!TIP]
> 建议同时在 `local_costmap` 和 `global_costmap` 中添加本层。局部代价地图用于实时避障，全局代价地图用于路径规划绕行。

## 编译

```bash
colcon build --packages-select teammate_cost_layer
```

依赖：
- `rclcpp` / `rclcpp_lifecycle` / `pluginlib`
- `nav2_costmap_2d` / `nav2_util`
- `rm_referee_msgs`

## 文件结构

```
teammate_cost_layer/
├── CMakeLists.txt
├── package.xml
├── teammate_cost_layer_plugin.xml        # pluginlib 导出描述
├── include/teammate_cost_layer/
│   └── teammate_cost_layer.hpp           # TeammateCostLayer 头文件
└── src/
    └── teammate_cost_layer.cpp           # 实现
```

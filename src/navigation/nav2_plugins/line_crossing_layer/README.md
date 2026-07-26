# line_crossing_layer

Nav2 costmap 插件层。在预定义线段两侧创建对称代价带，引导 A* 规划器选择**垂直穿越**路径。与 `pri_adaptive_mppi` 配合使用（共享相同的直线定义格式），实现"感知直线 → 规划垂直穿越 → 自适应 MPPI 跨越"的完整流程。

## 原理

```
                    代价带 (band_width)
              ←────────────────────────────→
                      
    ─────────────────●─────────────────●───  直线 segment
                      
              ←────────────────────────────→
                      
  垂直穿越: 穿过 2×band_width → 代价最小
  斜向穿越: 穿过更长距离      → 代价更大
  端点圆:  corner_radius 内径向梯度代价 → 阻止拐角穿越
```

A* 的代价感知边权重使路径自然偏向穿越距离最短的方向（即垂直方向）。

## 插件信息

| 属性 | 值 |
|------|-----|
| 插件类 | `line_crossing_layer::LineCrossingLayer` |
| 基类 | `nav2_costmap_2d::Layer` |
| 插件名（yaml 配置） | `line_crossing_layer::LineCrossingLayer` |

## 参数

### 基本参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `enabled` | bool | true | 是否启用 |
| `band_width` | float | 0.5 | 单侧代价带宽度 (m) |
| `cost_value` | int | 200 | 代价带内代价值 (0–254)，不会覆盖已存在的更高代价 |
| `lines` | string[] | `[]` | 直线名称列表 |

### 端点拐角代价（防止从线段端点绕行）

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `corner_radius` | float | 0.3 | 端点拐角代价圆半径 (m)，0 = 禁用 |
| `corner_cost_value` | int | 253 | 拐角中心峰值代价 (0–254) |
| `corner_gradient_power` | float | 1.0 | 径向梯度衰减指数（1=线性, 2=二次, 0.5=凹） |

### 直线定义（每条直线以 `<name>.point_1` / `<name>.point_2` 指定）

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `<name>.point_1` | float[2] | `[0.0, 0.0]` | 端点 1 坐标 |
| `<name>.point_2` | float[2] | `[5.0, 0.0]` | 端点 2 坐标 |

所有端点自动去重（1mm 容差），每条直线端点处的重复点只保留一个拐角代价圆。

## 在 costmap 中使用

与 `pri_adaptive_mppi` 共享相同的直线定义：

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["static_layer", "line_crossing", "inflation_layer"]

      line_crossing:
        plugin: "line_crossing_layer::LineCrossingLayer"
        enabled: true
        band_width: 0.5
        cost_value: 200
        corner_radius: 0.3
        corner_cost_value: 253
        corner_gradient_power: 1.0

        lines: ["ramp_a", "ramp_b"]

        ramp_a.point_1: [0.0, 0.0]
        ramp_a.point_2: [5.0, 0.0]

        ramp_b.point_1: [3.0, 0.0]
        ramp_b.point_2: [8.0, 0.0]
```

> [!TIP]
> 直线定义格式与 `pri_adaptive_mppi` 完全一致，可以将同一份 `lines` 配置复制到 costmap 层和 MPPI 控制器参数中，减少维护成本。

## 编译

```bash
colcon build --packages-select line_crossing_layer
```

依赖：
- `rclcpp` / `rclcpp_lifecycle` / `pluginlib`
- `nav2_costmap_2d` / `nav2_util`
- `tf2_ros`

## 文件结构

```
line_crossing_layer/
├── CMakeLists.txt
├── package.xml
├── line_crossing_layer_plugin.xml       # pluginlib 导出描述
├── include/line_crossing_layer/
│   └── line_crossing_layer.hpp          # LineCrossingLayer + LineDef 定义
└── src/
    └── line_crossing_layer.cpp          # 实现
```

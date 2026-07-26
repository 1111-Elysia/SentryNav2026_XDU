# cpp_lidar_filter

基于 PCL 的激光雷达点云裁剪过滤节点。主要用于去除 LiDAR 扫描中落在机器人自身车体范围内的点，避免下游 SLAM / 感知模块将自车误判为障碍物。同时发布裁剪区域的可视化 Marker，便于在 RViz 中确认过滤范围。

## 工作原理

1. 订阅输入点云话题（默认 `/livox/lidar`）
2. 使用 PCL `CropBox` 滤波器，在 `livox_frame` 坐标系下按配置的包围盒裁剪点云
   - `negative = true`（默认）：**剔除包围盒内的点**（即挖掉车身区域）
   - `negative = false`：**保留包围盒内的点**（仅关注特定区域）
3. 发布过滤后的点云到输出话题（默认 `/livox/lidar_filtered`）
4. 以 1 Hz 频率发布裁剪包围盒的 `Marker`（红色半透明立方体），便于在 RViz 中可视化调整

## 编译

```bash
# 在 ROS 2 工作空间中
colcon build --packages-select cpp_lidar_filter
```

依赖：
- `rclcpp`
- `sensor_msgs`
- `pcl_conversions` / `pcl_ros`
- `PCL` (common, io, filters)
- `visualization_msgs`

## 使用

```bash
ros2 run cpp_lidar_filter lidar_filter_node
```

默认参数下：订阅 `/livox/lidar`，发布到 `/livox/lidar_filtered`，裁剪盒为 `[-0.5, 0.5] × [-0.5, 0.5] × [-0.5, 0.6]`（米）。

带自定义参数启动：

```bash
ros2 run cpp_lidar_filter lidar_filter_node --ros-args \
  -p input_topic:=/livox/lidar \
  -p output_topic:=/livox/lidar_filtered \
  -p min_x:=-0.3 \
  -p max_x:=0.3 \
  -p min_y:=-0.3 \
  -p max_y:=0.3 \
  -p min_z:=-0.4 \
  -p max_z:=0.5 \
  -p negative:=true
```

## 话题

| 话题 | 方向 | 类型 | 说明 |
|------|------|------|------|
| `input_topic`（参数配置） | 订阅 | `sensor_msgs/msg/PointCloud2` | 原始点云输入 |
| `output_topic`（参数配置） | 发布 | `sensor_msgs/msg/PointCloud2` | 过滤后点云输出 |
| `/crop_box_marker` | 发布 | `visualization_msgs/msg/Marker` | 裁剪盒可视化（1 Hz） |

## 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `input_topic` | string | `"/livox/lidar"` | 输入点云话题 |
| `output_topic` | string | `"/livox/lidar_filtered"` | 输出点云话题 |
| `min_x` | float | -0.5 | 裁剪包围盒 X 最小值（米） |
| `max_x` | float | 0.5 | 裁剪包围盒 X 最大值（米） |
| `min_y` | float | -0.5 | 裁剪包围盒 Y 最小值（米） |
| `max_y` | float | 0.5 | 裁剪包围盒 Y 最大值（米） |
| `min_z` | float | -0.5 | 裁剪包围盒 Z 最小值（米） |
| `max_z` | float | 0.6 | 裁剪包围盒 Z 最大值（米） |
| `negative` | bool | true | `true` = 剔除盒内点；`false` = 保留盒内点 |
| `leaf_size` | float | 0.05 | 体素降采样边长（**当前代码中已注释，未生效**） |

> [!NOTE]
> - 所有参数均为 ROS 2 动态参数，运行时可通过 `ros2 param set` 实时调整
> - 裁剪盒坐标参考系为 `livox_frame`，Marker 也发布在 `livox_frame` 下
> - `negative = true` 是典型用法：挖掉车体在 LiDAR 视野中产生的自身点云

## 参数调优建议

### 车身裁剪（CropBox）

根据机器人实际尺寸和 LiDAR 安装位置设置：

| 机器人尺寸参考 | 建议参数 |
|--------------|---------|
| 小型哨兵 (~0.3m³) | `min_x=-0.2, max_x=0.2, min_y=-0.2, max_y=0.2, min_z=-0.3, max_z=0.4` |
| 中型底盘 (~0.5m³) | `min_x=-0.35, max_x=0.35, min_y=-0.35, max_y=0.35, min_z=-0.4, max_z=0.5` |

- **保守裁剪**：盒子略小于车体，避免误删有用点（允许少量自车点残留）
- **激进裁剪**：盒子略大于车体，彻底清除自车点（可能误删近距离障碍物）

> [!TIP]
> 启动后在 RViz 中添加 `/crop_box_marker` 的 Marker 显示，可直观看到裁剪盒位置和大小，方便调整参数。

## 文件结构

```
cpp_lidar_filter/
├── CMakeLists.txt          # ament_cmake 构建脚本
├── package.xml
└── src/
    └── filter_node.cpp     # 过滤节点源码
```

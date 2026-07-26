# costmap_intensity

Nav2 costmap 插件层，在标准障碍物层和体素层的基础上增加了**基于点云强度 (intensity) 的过滤**。通过设置强度阈值，可以过滤低强度噪点或高反射率伪影，避免将它们误标记为障碍物。

## 插件列表

| 插件类 | 基类 | 插件名（用于 yaml 配置） |
|--------|------|--------------------------|
| `costmap_intensity::ObstacleLayerIntensity` | `nav2_costmap_2d::Layer` | `costmap_intensity::ObstacleLayerIntensity` |
| `costmap_intensity::VoxelLayerIntensity` | `costmap_intensity::ObstacleLayerIntensity` | `costmap_intensity::VoxelLayerIntensity` |

## 与标准 Nav2 层的关键区别

标准的 `ObstacleLayer` / `VoxelLayer` 对所有点云一视同仁：有点即障碍物。本插件在 `updateBounds()` 中额外读取每个点的 `intensity` 字段：

- **ObstacleLayerIntensity**：强度不在 `[min_obstacle_intensity, max_obstacle_intensity]` 区间内的点被**丢弃**，不标记为障碍物
- **VoxelLayerIntensity**：强度高于 `max_obstacle_intensity` 的点被丢弃

适用场景：
- 过滤 LiDAR 在灰尘、雨雾中产生的低强度噪点
- 过滤高反射率表面（如玻璃、反光条）产生的伪影
- 在 RoboMaster 场地中过滤特定材质的 false positive 检测

## 新增参数

### ObstacleLayerIntensity

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `min_obstacle_intensity` | float | 0.0 | 障碍物最小强度，低于此值的点丢弃 |
| `max_obstacle_intensity` | float | 2.0 | 障碍物最大强度，高于此值的点丢弃 |

### VoxelLayerIntensity

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `max_obstacle_intensity` | float | 2.0 | 障碍物最大强度，高于此值的点丢弃 |

其余参数与标准 Nav2 `ObstacleLayer` / `VoxelLayer` 保持一致，参考 [Nav2 Costmap 配置文档](https://navigation.ros.org/configuration/packages/configuring-costmaps.html)。

## 在 costmap 中使用

在 local / global costmap 参数中，将 `plugin` 替换为本包提供的插件名：

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["obstacle_layer", "inflation_layer"]

      obstacle_layer:
        plugin: "costmap_intensity::ObstacleLayerIntensity"  # 替换标准 ObstacleLayer
        enabled: true
        min_obstacle_intensity: 0.1    # 过滤强度 < 0.1 的低强度噪点
        max_obstacle_intensity: 200.0  # 过滤强度 > 200 的反射伪影
        observation_sources: scan
        scan:
          topic: /scan
          data_type: "LaserScan"
          marking: true
          clearing: true
```

或者使用体素层：

```yaml
      voxel_layer:
        plugin: "costmap_intensity::VoxelLayerIntensity"  # 替换标准 VoxelLayer
        enabled: true
        max_obstacle_intensity: 200.0
        z_voxels: 16
        z_resolution: 0.05
        ...
```

## 编译

```bash
colcon build --packages-select costmap_intensity
```

## 体素可视化

在 RViz 中查看 3D 体素：

1. 确保 `voxel_layer` 参数中 `publish_voxel_map` 设为 `True`
2. 运行体素标记节点：

   ```bash
   ros2 run costmap_intensity costmap_intensity_markers \
     voxel_grid:=/local_costmap/voxel_grid \
     visualization_marker:=/voxel_marker
   ```

3. 在 RViz 中添加 `/voxel_marker` 的 Marker 显示
4. 将 RViz 视图切换为 3D (Orbit)，并将 `fixed_frame` 设为 `odom`

## 文件结构

```
costmap_intensity/
├── CMakeLists.txt
├── package.xml
├── costmap_intensity.xml              # pluginlib 插件导出描述
├── include/costmap_intensity/
│   ├── obstacle_layer.hpp             # ObstacleLayerIntensity 头文件
│   └── voxel_layer.hpp                # VoxelLayerIntensity 头文件
└── plugins/
    ├── obstacle_layer.cpp             # 障碍物层实现（含 intensity 过滤）
    └── voxel_layer.cpp                # 体素层实现（含 intensity 过滤）
```

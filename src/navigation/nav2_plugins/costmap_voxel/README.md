# costmap_voxel

Nav2 costmap 插件层。直接订阅 PointCloud2 点云话题，经 PCL 体素降采样和强度过滤后，将剩余点标记为 `LETHAL_OBSTACLE`。相比 `costmap_intensity` 中的 `ObstacleLayerIntensity` / `VoxelLayerIntensity`，本层更轻量：不做 ray-tracing 清除自由空间，不依赖 observation buffer，仅做标记。

## 处理流程

```
PointCloud2 (/livox/lidar/pointcloud)
         │
    ┌────▼────┐  PCL VoxelGrid 体素降采样
    │ 阶段 1  │  降低点密度，减少计算量
    └────┬────┘
         │
    ┌────▼────┐  强度过滤
    │ 阶段 2  │  丢弃 intensity > max_obstacle_intensity 的点
    └────┬────┘
         │
    ┌────▼────┐  标记
    │ 阶段 3  │  将剩余点写入 costmap (LETHAL_OBSTACLE)
    └─────────┘
```

## 插件信息

| 属性 | 值 |
|------|-----|
| 插件类 | `costmap_voxel::VoxelFilterLayer` |
| 基类 | `nav2_costmap_2d::Layer` |
| 插件名（yaml 配置） | `costmap_voxel::VoxelFilterLayer` |

## 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `<layer_name>.enabled` | bool | true | 是否启用本层 |
| `<layer_name>.voxel_leaf_size` | float | 0.1 | 体素降采样边长 (m)，越小保留越多细节 |
| `<layer_name>.max_obstacle_intensity` | float | 2.0 | 障碍物最大强度，高于此值的点丢弃 |
| `<layer_name>.pointcloud.topic` | string | `/livox/lidar/pointcloud` | 订阅的点云话题 |
| `<layer_name>.pointcloud.marking` | bool | true | 是否标记障碍物 |
| `<layer_name>.pointcloud.clearing` | bool | true | 是否清除（**当前实现中未使用**） |

## 在 costmap 中使用

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["voxel_filter", "inflation_layer"]

      voxel_filter:
        plugin: "costmap_voxel::VoxelFilterLayer"
        enabled: true
        voxel_leaf_size: 0.05           # 5cm 体素
        max_obstacle_intensity: 150.0   # 过滤高强度反射点
        pointcloud:
          topic: /livox/lidar/pointcloud
          marking: true
          clearing: false

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        ...
```

## 与 costmap_intensity 的对比

| | costmap_voxel | costmap_intensity |
|------|------|------|
| 基类 | `Layer` | `CostmapLayer` / `ObstacleLayerIntensity` |
| 点云获取 | 直接订阅 PointCloud2 | 通过 observation buffer（支持 LaserScan 转 PointCloud2） |
| 体素降采样 | ✅ 内置 PCL VoxelGrid | ❌（VoxelLayer 用体素占用，非降采样） |
| ray-tracing 清除 | ❌ 不做 | ✅ 做 |
| footprint 清除 | ❌ 不做 | ✅ 做 |
| 强度过滤 | `> max` 丢弃 | `[min, max]` 区间外丢弃 |
| 复杂度 | 轻量 | 完整（继承 Nav2 障碍物层全部逻辑） |

适用场景：
- **costmap_voxel**：只需简单地将点云标记为障碍物，场景简单，点密度高需要降采样
- **costmap_intensity**：需要完整的障碍物层功能（动态清除、ray-tracing、多传感器融合、LaserScan 支持）

## 编译

```bash
colcon build --packages-select costmap_voxel
```

依赖：
- `rclcpp` / `pluginlib`
- `sensor_msgs` / `pcl_conversions`
- `nav2_costmap_2d`
- `PCL` (common, io, filters)

## 文件结构

```
costmap_voxel/
├── CMakeLists.txt
├── package.xml
├── resource/
│   ├── costmap_voxel                   # ament index 标记文件
│   └── costmap_voxel_plugins.xml       # pluginlib 插件导出描述
├── include/costmap_voxel/
│   └── voxel_filter_layer.hpp          # VoxelFilterLayer 头文件
└── src/
    └── voxel_filter_layer.cpp          # 实现
```

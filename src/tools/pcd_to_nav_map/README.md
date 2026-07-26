# pcd_to_nav_map

将 PCD 格式的 3D 点云转换为 Nav2 兼容的 2D 占据栅格地图（PGM + YAML）。适用于通过 SLAM / LiDAR 建图得到点云后，生成可用于导航的静态地图。

该工具为独立的 CMake 项目（标记了 `COLCON_IGNORE`，不参与 colcon 构建），依赖 PCL 和 nlohmann_json。

## 工作流程

1. 加载 PCD 点云
2. 按配置的 roll / pitch / yaw 角度旋转点云（补偿 LiDAR 安装倾角）
3. 按 `min_z_height` ~ `max_z_height` 对点云做高度过滤，剔除地面和天花板
4. 体素下采样，降低点密度，提高后续处理效率
5. 以配置的 LiDAR 原点为起点，对每个点做 ray-casting（Bresenham）：射线经过的栅格标记为 Free，终点处标记为 Occupied
6. 输出 PGM 图像和 Nav2 YAML 地图描述文件

## 编译

```bash
cd src/tools/pcd_to_nav_map
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

编译产物为 `build/pcd_to_nav2_map` 可执行文件。

依赖：
- **PCL** ≥ 1.8 (`libpcl-dev`)
- **nlohmann_json** (`nlohmann-json3-dev`)

## 使用

```bash
./build/pcd_to_nav2_map <点云.pcd> <config.json>
```

示例：

```bash
./build/pcd_to_nav2_map ~/maps/lobby.pcd config/config.json
```

运行后在 `output_prefix` 指定的路径下生成两个文件：
- `<prefix>.pgm` — 占据栅格图像
- `<prefix>.yaml` — Nav2 map_server 可加载的地图描述文件

## 配置文件说明

配置文件为 JSON 格式，参考 [config/config.json](config/config.json) 和 [config/README.md](config/README.md)。所有参数及作用如下：

| 参数 | 类型 | 说明 |
|------|------|------|
| `output_prefix` | string | 输出文件前缀，最终生成 `<prefix>.pgm` 和 `<prefix>.yaml` |
| `resolution` | float | 地图像素分辨率（米/像素），常用 0.05 |
| `min_z_height` | float | 点云 Z 轴下界（米），低于此高度的点被丢弃 |
| `max_z_height` | float | 点云 Z 轴上界（米），高于此高度的点被丢弃 |
| `voxel_leaf_size` | float | 体素下采样边长（米），越小保留越多细节，但计算更慢 |
| `occupied_value` | int | PGM 中占据（障碍物）像素值，默认 0（黑） |
| `free_value` | int | PGM 中空闲像素值，默认 255（白） |
| `unknown_value` | int | PGM 中未知像素值，默认 255（白，与空闲相同） |
| `occupied_thresh` | float | Nav2 占据判定阈值，像素值 ≥ 此值视为占据 |
| `free_thresh` | float | Nav2 空闲判定阈值，像素值 ≤ 此值视为空闲 |
| `map_padding` | float | 地图边界外扩（米），避免规划器贴边 |
| `lidar_roll_deg` | float | LiDAR 绕 X 轴旋转补偿角（度） |
| `lidar_pitch_deg` | float | LiDAR 绕 Y 轴旋转补偿角（度） |
| `lidar_yaw_deg` | float | LiDAR 绕 Z 轴旋转补偿角（度） |
| `lidar_origin_x` | float | LiDAR 原点 X 坐标（米），ray-casting 的射线起点 |
| `lidar_origin_y` | float | LiDAR 原点 Y 坐标（米），ray-casting 的射线起点 |

### 参数调优建议

- **高度过滤**：`min_z_height` 应略高于地面，`max_z_height` 应低于天花板。典型值如 `0.45 ~ 2.0`。
- **体素下采样**：对于稠密点云（如 Livox），`voxel_leaf_size` 可设 0.05 ~ 0.1；稀疏点云设 0.001（几乎不过滤）。
- **occupied_thresh / free_thresh**：通常与像素值配合。若 `unknown_value == free_value`（均为 255），则将 `free_thresh` 设为一个接近 1.0 的值（如 0.99），以避免把未知区域误判为空闲。
- **LiDAR 旋转补偿**：如果 LiDAR 不是水平安装，调整对应角度将点云旋转至水平面。

## 在 Nav2 中加载生成的地图

生成的 `.yaml` 文件可直接被 `nav2_map_server` 加载。项目中提供了示例 launch 文件 [launch/test_map.launch.py](launch/test_map.launch.py)，启动方式：

```bash
ros2 launch pcd_to_nav_map test_map.launch.py
```

该 launch 文件会同时启动：
- `nav2_lifecycle_manager` — 管理节点生命周期
- `nav2_map_server` — 加载静态地图
- `nav2_costmap_2d` — 发布 global costmap
- `tf2_ros static_transform_publisher` — 发布 map → base_link 静态 TF（测试用）

> [!NOTE]
> `test_map.launch.py` 中的 `params_file` 路径是硬编码的绝对路径，使用前请修改为实际 `.yaml` 文件路径。

## 文件结构

```
pcd_to_nav_map/
├── CMakeLists.txt          # CMake 构建脚本
├── COLCON_IGNORE           # 标记不参与 colcon 构建
├── README.md
├── config/
│   ├── config.json         # 示例配置文件
│   ├── README.md           # 配置文件详细说明
│   └── test_map.yaml       # Nav2 map_server 参数示例
├── include/
│   └── json.hpp            # nlohmann_json 单头文件（备用）
├── launch/
│   └── test_map.launch.py  # Nav2 地图加载 launch 示例
├── src/
│   └── pcd_to_nav_map.cpp  # 主程序
└── build/                  # 编译产物目录
```

## 算法说明

### Ray-casting（Bresenham 直线光栅化）

以 LiDAR 原点为射线起点，对每个点云点沿射线路径标记栅格：

- **射线经过的栅格** → 标记为 Free（空闲），但**不会覆盖**已标记为 Occupied（占据）的栅格，避免密集点云把障碍物“洗掉”
- **射线终点（点云点所在栅格）** → 标记为 Occupied（占据）

### 高度过滤

点云的 Z 坐标在 `min_z_height` ~ `max_z_height` 之外的直接丢弃。这能剔除地面点和天花板/顶棚点，只保留墙体、障碍物等对导航有意义的结构。

### 体素下采样

使用 PCL 的 `VoxelGrid` 滤波器对高度过滤后的点云做降采样，减少 ray-casting 的射线数量，大幅提升处理速度。

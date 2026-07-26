# d435_process_pcl

将 Intel RealSense D435 深度图像转换为 3D 点云（PointCloud2）的 ROS 2 节点。内置**多级滤波管线**（空间中值滤波 → 时域中值滤波 → 针孔投影 → 体素降采样 → 统计离群点剔除），专门针对机器人场景中的 D435 噪声特征（散粒噪点、水面波动、IR 内反射伪影）进行了优化。

## 处理管线

```
深度图 (16U / 32FC1) + CameraInfo
         │
    ┌────▼────┐  空间中值滤波 (medianBlur)，去散粒噪点
    │ 阶段 1  │  保护无效像素 (depth==0) 不扩散
    └────┬────┘
         │
    ┌────▼────┐  时域中值滤波 (滑动窗口 N 帧)
    │ 阶段 2  │  逐像素取时域中值，抑制水面/反光面波动
    └────┬────┘
         │
    ┌────▼────┐  针孔投影 (depth → 3D)
    │ 阶段 3  │  + step 下采样  + top_margin 跳过顶部伪影
    │         │  + min/max distance 距离过滤
    └────┬────┘
         │
    ┌────▼────┐  VoxelGrid 体素降采样
    │ 阶段 4  │  均匀空间降采样，减少点密度
    └────┬────┘
         │
    ┌────▼────┐  StatisticalOutlierRemoval (SOR)
    │ 阶段 5  │  去除离群飞点
    └────┬────┘
         │
         ▼
   PointCloud2 发布
```

## 编译

```bash
colcon build --packages-select d435_process_pcl
```

依赖：
- `rclcpp` / `sensor_msgs`
- `cv_bridge` / `image_geometry`
- `pcl_conversions` / `pcl_ros` / `PCL` (common, filters)
- `message_filters` (深度图 + CameraInfo 时间同步)

## 使用

### Launch 启动

```bash
ros2 launch d435_process_pcl depth_to_pcl.launch.py
```

### 单节点启动

```bash
ros2 run d435_process_pcl depth_to_pcl_node --ros-args --params-file config/params.yaml
```

## 话题

| 话题 | 方向 | 类型 | 说明 |
|------|------|------|------|
| `/camera/camera/depth/image_rect_raw` | 订阅 | `sensor_msgs/msg/Image` | D435 深度图输入 |
| `/camera/camera/depth/camera_info` | 订阅 | `sensor_msgs/msg/CameraInfo` | 相机内参 |
| `d435_pointcloud`（可配置） | 发布 | `sensor_msgs/msg/PointCloud2` | 滤波后的点云输出 |

### TF 广播

| 父帧 | 子帧 | 说明 |
|------|------|------|
| `base_link` | `d435_frame` | 相机在机器人上的安装位置（静态 TF，由参数配置） |

## 参数

配置文件：[config/params.yaml](config/params.yaml)

### 话题与坐标系

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `output_topic` | string | `"d435_pointcloud"` | 输出点云话题名 |
| `depth_image_topic` | string | `"/camera/camera/depth/image_rect_raw"` | 深度图输入话题 |
| `camera_info_topic` | string | `"/camera/camera/depth/camera_info"` | 相机内参话题 |
| `frame_id` | string | `"d435_frame"` | 输出点云的 frame_id |

### 静态 TF（`base_link` → `d435_frame`）

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `tf_x` | float | -0.12 | 相机安装位置 X（米） |
| `tf_y` | float | 0.0 | 相机安装位置 Y（米） |
| `tf_z` | float | 0.0 | 相机安装位置 Z（米） |
| `tf_roll_deg` | float | -90.0 | 相机安装 roll 角（度） |
| `tf_pitch_deg` | float | 0.0 | 相机安装 pitch 角（度） |
| `tf_yaw_deg` | float | 90.0 | 相机安装 yaw 角（度） |

> [!NOTE]
> D435 的默认安装姿态 `roll=-90°, yaw=90°` 用于将摄像头从水平朝前姿态旋转到 RGB 镜头朝下的典型安装方向。

### 投影与下采样

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `step` | int | 2 | 像素步长，step=2 即每隔一行一列采一个点（点数降至 1/4） |
| `top_margin` | int | 30 | 跳过图像顶部行数，避免 D435 上视场角 IR 内反射伪影 |
| `min_distance` | float | 0.2 | 最小有效深度（米），小于此值丢弃 |
| `max_distance` | float | 3.0 | 最大有效深度（米），大于此值丢弃 |

### 滤波参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `median_kernel_size` | int | 5 | 空间中值滤波核大小（必须为奇数），去散粒噪点 |
| `temporal_frames` | int | 5 | 时域中值滤波帧数（1 = 禁用），抑制水面波动 |
| `voxel_leaf_size` | float | 0.03 | 体素降采样边长（米） |
| `sor_mean_k` | int | 20 | SOR 统计滤波的邻近点数量 |
| `sor_stddev_thresh` | float | 1.0 | SOR 标准差倍数阈值，越小越激进 |

## 调优建议

### 场景自适应

| 场景 | 建议调整 |
|------|---------|
| 室内静态（无水面） | `temporal_frames=1`（禁用时域滤波），降低延迟 |
| 室外/水面反光 | `temporal_frames=5~7`（增强时域平滑），压制水波闪烁点 |
| 近处避障 | `max_distance` 减小到 1.5~2.0m，减少远距离噪点 |
| 远处感知 | `max_distance` 增大，`voxel_leaf_size` 适当增大以控制点密度 |
| 低速场景 | `step=1`（全分辨率），`top_margin=0` |

### 性能优化

- **`step`** 是最有效的性能参数：`step=2` 点数降至 1/4，`step=3` 降至 1/9
- **`voxel_leaf_size`** 对稠密墙面有效：0.03m ~ 0.05m 平衡质量与速度
- **`temporal_frames`** 每增加 1 帧，额外占用一帧深度的内存

## RViz 可视化

项目提供了预配置的 RViz 布局文件：

```bash
rviz2 -d install/d435_process_pcl/share/d435_process_pcl/rviz/d435.rviz
```

## 文件结构

```
d435_process_pcl/
├── CMakeLists.txt                # ament_cmake 构建脚本
├── package.xml
├── config/
│   └── params.yaml              # 默认参数配置
├── launch/
│   └── depth_to_pcl.launch.py   # 一键启动 launch
├── rviz/
│   └── d435.rviz                # RViz 预配置文件
└── src/
    └── depth_to_pcl_node.cpp    # 转换节点源码
```

# 初始位姿检查器 (Init Pos Checker)

验证定位系统重定位初始化是否正确。**不需要手动测量任何东西。**

---

## 原理

定位系统的 `/lio/cloud_world` 话题输出的点云**已经被变换到 map 坐标系下**。
如果定位准确，这些点云应该和建图时保存的地图 PCD 完全重合。

本工具直接将 `/lio/cloud_world` 与地图 PCD 做 ICP 对齐：
- **ICP 修正量很小** → 定位准确，初始位姿配置正确
- **ICP 修正量很大** → 定位有偏差，初始位姿配置有问题

---

## 编译

```bash
cd /home/kyrie/SentryNav2026_XDU
colcon build --packages-select init_pos_checker
source install/setup.bash
```

---

## 完整测试流程

### 第一步：建图

在场地中启动建图，走完整个场地后保存地图。

```bash
# 终端 1：启动建图
ros2 launch bringup slam_online.launch.py

# 建图结束后，地图 PCD 保存在以下位置（根据你的实际路径）：
# /home/kyrie/SentryNav2026_XDU/data/map/
```

> 建图完成后记住地图保存路径，后面要用。

### 第二步：关闭建图

```bash
# Ctrl+C 关闭建图节点
```

### 第三步：移动车辆

将车移动到场地中**任意一个新位置**，可以旋转任意角度。
**不需要测量位移，不需要记录角度。**

### 第四步：配置初始位姿

确保 `bringup/config/loc_start_pose.yaml` 中填写了车辆当前位置对应的初始位姿。
这是你的定位系统启动时使用的初始猜测值。

### 第五步：启动定位系统

```bash
# 终端 1：启动定位
ros2 launch bringup loc_online.launch.py
```

等待几秒，确认定位系统正常运行。可以用以下命令确认点云话题有数据：

```bash
# 终端 2：确认话题有数据
ros2 topic hz /lio/cloud_world
# 应该看到有频率输出，例如 10 Hz
```

### 第六步：启动检查器

```bash
# 终端 2：启动检查器（指定地图路径）
ros2 launch init_pos_checker pose_checker.launch.py \
  map_pcd_path:=/home/kyrie/SentryNav2026_XDU/data/map/
```

> ⚠️ `map_pcd_path` 必须指定！可以是：
> - 单个 `.pcd` 文件路径
> - 包含多个 `.pcd` 文件的目录路径

### 第七步：查看终端输出

检查器会自动累积 5 帧点云，然后输出结果：

**✅ 定位正确的输出示例：**

```
========================================
  ICP 对齐验证
========================================

收敛:       ✓ 是
适应度:     0.028456

位置修正:
  dx = 0.0230 m
  dy = -0.0150 m
  dz = 0.0080 m
  总计 = 0.0290 m

旋转修正:
  roll  = 0.500°
  pitch = 0.300°
  yaw   = 1.200°
  总计  = 1.328°

----------------------------------------
✅ 定位准确！点云与地图对齐良好
   位置误差 0.0290 m < 阈值 0.50 m
   旋转误差 1.328° < 阈值 5.00°
========================================
```

**❌ 定位有偏差的输出示例：**

```
----------------------------------------
❌ 定位存在偏差！
   位置: 1.2340 m > 阈值 0.50 m
   旋转: 8.500° > 阈值 5.00°

   可能原因:
   1. loc_start_pose.yaml 中的初始位姿不准确
   2. lidar.yaml 中的外参不正确
   3. 地图与当前环境不匹配
========================================
```

### 第八步：RViz 可视化（可选）

```bash
# 终端 3
rviz2
```

在 RViz 中配置：

1. **Fixed Frame** 设为 `map`
2. 添加 3 个 **PointCloud2** 显示：

| 话题 | 建议颜色 | 含义 |
|------|----------|------|
| `/debug/map_cloud` | 绿色 | 建图保存的地图 |
| `/debug/current_cloud` | 红色 | 定位系统输出的当前点云 |
| `/debug/aligned_cloud` | 白色 | ICP 对齐后的点云 |

**判断方法：**

| 现象 | 结论 |
|------|------|
| 红色和绿色重合 | ✅ 定位正确 |
| 白色和绿色重合，但红色偏移 | ❌ 定位有偏差，ICP 修正了 |
| 三者都不重合 | ❌ 定位完全失败 |

---

## Launch 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `map_pcd_path` | `""` | **必填**，地图 PCD 路径（文件或目录） |
| `cloud_topic` | `/lio/cloud_world` | 点云话题（已在 map 坐标系下） |

## 节点参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `accumulate_frames` | `5` | 累积点云帧数（越多越稳定） |
| `voxel_size` | `0.3` | 降采样体素大小 (m) |
| `icp_max_iter` | `50` | ICP 最大迭代次数 |
| `pos_threshold` | `0.5` | 位置误差阈值 (m) |
| `rot_threshold` | `5.0` | 旋转误差阈值 (°) |

---

## 结果判定标准

| 指标 | ✅ 通过 | ⚠️ 需关注 | ❌ 失败 |
|------|---------|-----------|---------|
| 位置误差 | < 0.5 m | 0.5 ~ 1.0 m | > 1.0 m |
| 旋转误差 | < 5° | 5° ~ 10° | > 10° |
| ICP 适应度 | < 0.1 | 0.1 ~ 0.3 | > 0.5 |

---

## 常见问题

### 1. 报错 "未指定 map_pcd_path"

启动时必须指定地图路径：

```bash
ros2 launch init_pos_checker pose_checker.launch.py \
  map_pcd_path:=/home/kyrie/SentryNav2026_XDU/data/map/
```

### 2. 报错 "路径不存在"

检查地图路径是否正确，确认建图后地图已保存：

```bash
ls /home/kyrie/SentryNav2026_XDU/data/map/
# 应该看到 .pcd 文件
```

### 3. 一直显示 "等待点云数据"

确认定位系统已启动且点云话题有输出：

```bash
ros2 topic list | grep cloud_world
ros2 topic hz /lio/cloud_world
```

### 4. ICP 未收敛

- 增大 `accumulate_frames`（如改为 10）累积更多点云
- 减小 `voxel_size`（如改为 0.2）保留更多细节
- 增大 `icp_max_iter`（如改为 100）

```bash
ros2 launch init_pos_checker pose_checker.launch.py \
  map_pcd_path:=/home/kyrie/SentryNav2026_XDU/data/map/ \
  --ros-args -p accumulate_frames:=10 -p voxel_size:=0.2
```

### 5. 定位偏差较大

按以下顺序排查：

1. **检查 `loc_start_pose.yaml`**：初始位姿是否合理
2. **检查 `lidar.yaml`**：雷达外参是否正确
3. **检查地图**：地图是否完整覆盖当前位置
4. **检查环境**：场地是否发生了变化（移动了障碍物等）
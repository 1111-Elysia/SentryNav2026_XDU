# imu-calib

IMU 静态自动校准节点 — 通过卡尔曼滤波估计重力方向，计算 LiDAR 内置 IMU 的安装倾角（roll / pitch），并发布校准后的 IMU 数据。适用于 Livox 等自带 IMU 的激光雷达，在静止状态下自动标定出 IMU 坐标系与水平面的偏差。

该节点为 ROS 2 `ament_cmake` 包（标记了 `COLCON_IGNORE`，默认不参与 colcon 全量构建）。

## 工作原理

1. 订阅 `/livox/imu` 原始 IMU 数据
2. 以首帧加速度作为重力初值，启动**卡尔曼滤波器**持续估计重力向量
3. 前 **200 帧**为滤波器稳定期，不做采集
4. 稳定后采集 **500 帧**的重力估计值并取平均
5. 利用 `setFromTwoVectors` 计算从“测量重力方向”到“目标重力方向 (0, 0, 1)”的旋转四元数
6. 从旋转矩阵中提取 **roll** 和 **pitch** 角度，即为 IMU 安装倾角
7. 持续发布校准后的 IMU 数据到 `/livox/imu_calib`，其中：
   - `orientation` 填入校准旋转四元数
   - `linear_acceleration` 填入平均后的重力向量

## 依赖

- ROS 2 (rclcpp, sensor_msgs, geometry_msgs)
- tf2 / tf2_geometry_msgs
- Eigen3

## 编译

```bash
# 在 ROS 2 工作空间中
colcon build --packages-select imu-calib

## 使用

```bash
ros2 run imu-calib imu_calib_node
```

> [!IMPORTANT]
> 运行前确保机器人/雷达处于**静止状态**，且 `/livox/imu` 话题有数据发布。校准结果只在静态条件下有效。

## 话题

| 话题 | 方向 | 类型 | 说明 |
|------|------|------|------|
| `/livox/imu` | 订阅 | `sensor_msgs/msg/Imu` | 原始 IMU 数据输入 |
| `/livox/imu_calib` | 发布 | `sensor_msgs/msg/Imu` | 校准后的 IMU 数据输出 |

## 运行日志解读

```
[imu_calib_node] 卡尔曼滤波启动成功
[imu_calib_node] 采集中... 100/500
[imu_calib_node] 采集中... 200/500
[imu_calib_node] 采集中... 300/500
[imu_calib_node] 采集中... 400/500
[imu_calib_node] 采集中... 500/500
[imu_calib_node] 采集完成，开始锁定最终值
[imu_calib_node] 【最终锁定】Roll: -1.2345, Pitch: 2.3456
```

- 前 200 帧为滤波器收敛期，无日志输出
- 之后每 100 帧输出一次采集进度
- 500 帧采集完成后锁定最终值，每 50 帧输出一次 roll / pitch
- **最终锁定的 roll / pitch 即 IMU 安装倾角**，可作为 `pcd_to_nav_map` 等工具的 `lidar_roll_deg` / `lidar_pitch_deg` 参数

## 参数说明

当前参数硬编码于代码中，如需调整可在 `src/imu_auto_calib.cpp` 中修改：

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `stable_frames` 阈值 | 200 | 滤波器稳定帧数，此后开始采集 |
| `collect_count` 上限 | 500 | 重力向量采集帧数 |
| `process_noise_cov` | `1e-6 × I` | 卡尔曼滤波过程噪声协方差 |
| `measurement_noise_cov` | `5.0 × I` | 卡尔曼滤波测量噪声协方差 |
| `estimate_error_cov` 初值 | `1.0 × I` | 估计误差协方差初值 |

- **过程噪声小 (1e-6)**：假设真实重力方向在静态下不变
- **测量噪声大 (5.0)**：加速度计瞬时噪声较大，滤波器会平滑突变

## 如何将校准结果用于其他工具

校准得到的 roll / pitch 可直接填入 `bringup/config/lidar.yaml` `pcd_to_nav_map` 的配置文件，用于点云旋转补偿：

```json
{
  "lidar_roll_deg": -1.2345,
  "lidar_pitch_deg": 2.3456,
  "lidar_yaw_deg": 0.0
}
```

## 文件结构

```
imu-calib/
├── CMakeLists.txt          # ament_cmake 构建脚本
├── COLCON_IGNORE           # 标记不参与全量 colcon 构建
├── package.xml             # ROS 2 包描述
└── src/
    └── imu_auto_calib.cpp  # 校准节点源码
```

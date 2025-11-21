# Sentry Navigation 2026 XDU

ros2 topic pub /hurt_armor sentry_msgs/msg/HurtArmor "{hurt_armor: 2}" -r 1
ros2 topic pub /hurt_armor sentry_msgs/msg/HurtArmor "{hurt_armor: 0}" -1
ros2 topic pub /match_stage sentry_msgs/msg/MatchStage "{match_stage: 4}" -r 1


## 依赖安装

```bash
bash ./src/lightning-lm/scripts/install_dep.sh
```

## 编译

```bash
# 编译导航包
colcon build --packages-select sentry_navigation --cmake-args -DCMAKE_BUILD_TYPE=Release
# 编译 Livox 转换包
colcon build --packages-select livox_to_scan --cmake-args -DCMAKE_BUILD_TYPE=Release
# 或者编译所有包
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Livox 雷达转换

启动 Livox 点云转 LaserScan 节点（使用默认参数）：
```bash
ros2 run livox_to_scan livox_to_scan_node
```

启动并使用自定义参数文件：
```bash
ros2 run livox_to_scan livox_to_scan_node --ros-args --params-file src/livox_to_scan/config/livox_to_scan_params.yaml
```

参数说明：
- `min_height` / `max_height`: 高度过滤范围（米）
- `angle_min` / `angle_max`: 角度范围（弧度）
- `angle_increment`: 角度分辨率（弧度）
- `range_min` / `range_max`: 距离过滤范围（米）
- `scan_time`: 扫描周期（秒）

## Lightning-LM 建图与定位

### SLAM 建图模式
```bash
ros2 run lightning run_slam_online --config ./src/lightning-lm/config/default_nclt.yaml
```

### 保存地图
```bash
ros2 service call /lightning/save_map lightning/srv/SaveMap "{map_id: 'new_map'}"
```

### 定位模式
```bash
ros2 run lightning run_loc_online --config ./src/lightning-lm/config/default_nclt.yaml
```

## 启动导航

### 完整启动（包含地图服务器和导航栈）
```bash
ros2 launch sentry_navigation bringup_launch.py map:=./data/new_map/map.yaml
```

### 启动导航并显示 RViz
```bash
ros2 launch sentry_navigation navigation_launch.py map:=./data/new_map/map.yaml use_rviz:=true
```

### 仅启动导航栈（不含地图服务器）
```bash
ros2 launch sentry_navigation navigation_launch.py
```

### 启动导航栈并显示 RViz
```bash
ros2 launch sentry_navigation navigation_launch.py use_rviz:=true
```

### 调试：单独启动 RViz
如果 RViz 没有显示，可以单独启动：
```bash
rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

或使用自定义配置：
```bash
rviz2 -d $(ros2 pkg prefix sentry_navigation)/share/sentry_navigation/rviz/nav2_view.rviz
```

## 故障排查

### RViz 中地图距离机器人很远

1. **检查 TF 树**：
```bash
ros2 run tf2_tools view_frames
# 会生成 frames.pdf 文件，检查 map -> odom -> base_link 的连接
```

2. **查看 TF 变换**：
```bash
ros2 run tf2_ros tf2_echo map base_link
```

3. **检查地图原点**：
检查地图 yaml 文件中的 `origin` 参数，确保合理。例如：
```yaml
origin: [0.0, 0.0, 0.0]
```

4. **在 RViz 中调整视角**：
   - 将 `Target Frame` 设置为 `base_link` 而不是 `map`
   - 按下键盘 `F` 键聚焦到选中的物体
   - 使用鼠标滚轮调整视距

5. **重置 RViz 视角**：
   - Views -> Current View -> Reset
   - 或者点击工具栏的 "FocusCamera" 工具，然后点击机器人模型

### TF 调试命令

查看所有 TF 帧：
```bash
ros2 run tf2_ros tf2_monitor
```

查看特定变换：
```bash
# 查看 map 到 base_link 的变换（由 lightning-lm 发布）
ros2 topic echo /tf --field transforms

# 实时查看变换
ros2 run tf2_ros tf2_echo map base_link
```

### 确认 lightning-lm 是否正常工作

```bash
# 查看是否发布了 base_link -> map 的 tf
ros2 run tf2_tools view_frames

# 检查定位是否正常
ros2 topic hz /tf
ros2 topic echo /tf | grep base_link
```

## 架构说明

- **定位**: lightning-lm **只**发布 `map -> base_link` 的 tf
- **里程计**: 导航系统发布静态 tf `map -> odom`（单位变换，使 odom 和 map 重合）
- **传感器**: Livox 雷达，话题 `/livox/lidar`
- **导航框架**: ROS 2 Navigation2 (Nav2)

## TF 树结构

**当前方案（lightning-lm 未发布 map 时）**：


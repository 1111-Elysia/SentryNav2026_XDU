# Sentry Navigation 2026 XDU


## 依赖安装

```bash
bash ./src/lightning-lm/scripts/install_dep.sh
```

## 编译
编译前需要source你的ws_livox，该框架不包含雷达驱动

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Lightning-LM 建图与定位

### SLAM 建图模式
```bash
ros2 run lightning run_slam_online --config ./src/lightning-lm/config/default_livox.yaml
```

### 保存地图
```bash
ros2 service call /lightning/save_map lightning/srv/SaveMap "{map_id: 'new_map'}"
```

### 定位模式
```bash
ros2 run lightning run_loc_online --config ./src/lightning-lm/config/default_livox.yaml
```

## 启动导航

### 完整启动（包含地图服务器和导航栈）
```bash
bash test2.sh
```

### 调试命令

模拟裁判系统发布比赛阶段
```bash
ros2 topic pub /match_stage sentry_msgs/msg/MatchStage "{match_stage: 4}" -r 1
```

## 架构说明

- **定位**: lightning-lm **只**发布 `map -> base_link` 的 tf
- **里程计**: 导航系统发布静态 tf `map -> odom`（单位变换，使 odom 和 map 重合）
- **传感器**: Livox 雷达，话题 `/livox/lidar`
- **导航框架**: ROS 2 Navigation2 (Nav2)

## TF 树结构



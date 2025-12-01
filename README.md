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

- **传感器**: Livox 雷达，话题 `/livox/lidar` `/livox/imu`
- **重定位**: lightning-lm 发布 `map -> livox_frame` 的 tf
- **里程计**: fastlio 发布 `odom -> livox_frame` 的 tf
- **点云转2D激光**: livox_to_scan 发布 `/scan`
- **导航**: sentry_navigation 解算`map->odom`和`odom->base_link`，并启动nav2
- **通讯节点**: serial_comm 实现与下位机的双向通信
- **自定义消息包**: sentry_msgs
- **点云转栅格地图工具包**: pcd_to_nav_map

## TF 树结构

map--->odom--->base_link--->livox_frame

## TODO
### 1、新建bringup包，在launch中管理全节点生命周期
### 2、接入行为树测试
### 3、修改高翔，使可根据红蓝方发布初始位姿[done]
### 4、修改通信包和消息包，接入裁判系统
### 5、测试其他重定位算法，尝试使用多重定位提高稳定性
### 6、调整nav参数，使车辆运行顺畅
### 7、参考川大开源，编写恢复行为自定义插件



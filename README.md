# Sentry Navigation 2026 XDU


## 依赖安装

```bash
bash depend_install.sh
```

## 编译
编译前需要source你的ws_livox，该框架不包含雷达驱动

```bash
bash build.sh
```

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
ros2 run lightning run_loc_online --config ./src/bringup/config/default_livox.yaml
```

### 完整启动
```bash
bash start.sh
```


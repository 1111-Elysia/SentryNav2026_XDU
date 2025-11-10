#!/bin/bash

# 设置环境变量
SOURCE_CMD="source /opt/ros/humble/setup.bash && source ./install/setup.bash && source ../Workspace/ws_livox/install/setup.bash"

# 启动 Lightning-LM 定位
gnome-terminal -- bash -c "$SOURCE_CMD && ros2 run lightning run_loc_online --config ./src/lightning-lm/config/default_nclt.yaml; exec bash"

# 等待 1 秒
sleep 1

# 启动 Fast-LIO 里程计
gnome-terminal -- bash -c "$SOURCE_CMD && ros2 launch fast_lio mapping.launch.py; exec bash"

# 等待 1 秒
sleep 1

# 启动 Livox 转 LaserScan 节点
gnome-terminal -- bash -c "$SOURCE_CMD && ros2 run livox_to_scan livox_to_scan_node --ros-args --params-file install/livox_to_scan/share/livox_to_scan/config/livox_to_scan_params.yaml; exec bash"

# 等待 2 秒（确保传感器数据就绪）
sleep 2

# 启动导航栈和 RViz
gnome-terminal -- bash -c "$SOURCE_CMD && ros2 launch sentry_navigation navigation_launch.py map:=./data/new_map/map.yaml use_rviz:=true; exec bash"

echo "All nodes launched in separate terminals"

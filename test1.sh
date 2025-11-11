#!/bin/bash

# ===== 配置区域 =====
MAP_YAML="./data/new_map/map.yaml"

# ===== 颜色输出 =====
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# ===== 环境变量 =====
SOURCE_CMD="source /opt/ros/humble/setup.bash && source ./install/setup.bash && source ../ws_livox/install/setup.bash"

# ===== 检查地图文件 =====
if [ ! -f "$MAP_YAML" ]; then
    echo -e "${RED}错误: 地图文件不存在: $MAP_YAML${NC}"
    exit 1
fi

echo -e "${GREEN}使用地图: $MAP_YAML${NC}"

# ===== 启动节点 =====
echo -e "${GREEN}[1/5] 启动 Lightning-LM 定位节点...${NC}"
gnome-terminal --title="Lightning-LM" -- bash -c "$SOURCE_CMD && \
    ros2 run lightning run_loc_online \
    --config ./src/lightning-lm/config/default_nclt.yaml; \
    exec bash"

sleep 2

echo -e "${GREEN}[2/5] 启动 Fast-LIO (仅用于速度估计)...${NC}"
gnome-terminal --title="Fast-LIO" -- bash -c "$SOURCE_CMD && \
    ros2 launch fast_lio mapping.launch.py; \
    exec bash"

sleep 2

# 修改 [3/5] 部分
echo -e "${GREEN}[3/5] 启动 TF & Odom 发布节点 (Lightning-LM + 坐标校正)...${NC}"
gnome-terminal --title="TF-Odom-Publisher" -- bash -c "$SOURCE_CMD && \
    ros2 run sentry_navigation tf_odom_publisher \
    --ros-args \
    -p publish_rate:=50.0 \
    -p use_correction:=true \
    -p correction_x:=-13.4 \
    -p correction_y:=-11.0 \
    -p correction_z:=0.0 \
    -p base_link_to_livox_x:=0.1 \
    -p base_link_to_livox_y:=0.0 \
    -p base_link_to_livox_z:=0.0; \
    exec bash"

sleep 1

echo -e "${GREEN}[4/5] 启动 Livox 转 LaserScan 节点...${NC}"
gnome-terminal --title="Livox-to-Scan" -- bash -c "$SOURCE_CMD && \
    ros2 run livox_to_scan livox_to_scan_node \
    --ros-args \
    --params-file install/livox_to_scan/share/livox_to_scan/config/livox_to_scan_params.yaml; \
    exec bash"

sleep 2

echo -e "${GREEN}[5/5] 启动导航栈和 RViz...${NC}"
gnome-terminal --title="Navigation" -- bash -c "$SOURCE_CMD && \
    ros2 launch sentry_navigation navigation_launch.py \
    map:='$MAP_YAML' \
    use_rviz:=true; \
    exec bash"

echo -e "${GREEN}=====================================${NC}"
echo -e "${GREEN}所有节点已在独立终端启动${NC}"
echo -e "${GREEN}=====================================${NC}"
echo -e "${YELLOW}系统架构:${NC}"
echo -e "  位置: Lightning-LM 重定位 (map→odom)"
echo -e "  速度: Fast-LIO (/Odometry twist)"
echo -e "  TF树: map → odom → base_link → livox_frame"
echo -e ""
echo -e "${YELLOW}调试命令:${NC}"
echo -e "  1. 检查 TF 树:"
echo -e "     ${GREEN}ros2 run tf2_tools view_frames${NC}"
echo -e "  2. 查看机器人位置 (应在地图上):"
echo -e "     ${GREEN}ros2 run tf2_ros tf2_echo map base_link${NC}"
echo -e "  3. 查看 odom 话题 (位置=0, 速度≠0):"
echo -e "     ${GREEN}ros2 topic echo /odom${NC}"
echo -e "  4. 监控速度来源:"
echo -e "     ${GREEN}ros2 topic echo /Odometry/twist/twist${NC}"
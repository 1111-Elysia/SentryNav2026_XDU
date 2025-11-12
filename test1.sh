#!/bin/bash
# filepath: /home/pgd/SentryNav2026_XDU/test1.sh

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

echo -e "${GREEN}=====================================${NC}"
echo -e "${GREEN}  哨兵导航系统启动脚本${NC}"
echo -e "${GREEN}=====================================${NC}"
echo -e "${YELLOW}使用地图: $MAP_YAML${NC}"
echo ""

# ===== 启动节点 =====
echo -e "${GREEN}[1/7] 启动 Livox MID360 雷达驱动...${NC}"
gnome-terminal --title="Livox-Driver" -- bash -c "$SOURCE_CMD && \
    ros2 launch livox_ros_driver2 msg_MID360_launch.py; \
    exec bash"

sleep 3  # 等待雷达初始化

echo -e "${GREEN}[2/7] 启动 Lightning-LM 定位节点...${NC}"
gnome-terminal --title="Lightning-LM" -- bash -c "$SOURCE_CMD && \
    ros2 run lightning run_loc_online \
    --config ./src/lightning-lm/config/default_nclt.yaml; \
    exec bash"

sleep 2

echo -e "${GREEN}[3/7] 启动 Fast-LIO (用于速度估计)...${NC}"
gnome-terminal --title="Fast-LIO" -- bash -c "$SOURCE_CMD && \
    ros2 launch fast_lio mapping.launch.py; \
    exec bash"

sleep 2

echo -e "${GREEN}[4/7] 启动 TF & Odom 发布节点...${NC}"
gnome-terminal --title="TF-Odom-Publisher" -- bash -c "$SOURCE_CMD && \
    ros2 run sentry_navigation tf_odom_publisher \
    --ros-args \
    -p publish_rate:=50.0 \
    -p base_link_to_livox_x:=0.117 \
    -p base_link_to_livox_y:=0.0 \
    -p base_link_to_livox_z:=0.0 \
    -p base_link_to_livox_roll:=0.0 \
    -p base_link_to_livox_pitch:=0.0 \
    -p base_link_to_livox_yaw:=0.0; \
    exec bash"

sleep 1

echo -e "${GREEN}[5/7] 启动 Livox 转 LaserScan 节点...${NC}"
gnome-terminal --title="Livox-to-Scan" -- bash -c "$SOURCE_CMD && \
    ros2 run livox_to_scan livox_to_scan_node \
    --ros-args \
    --params-file install/livox_to_scan/share/livox_to_scan/config/livox_to_scan_params.yaml; \
    exec bash"

sleep 1

echo -e "${GREEN}[6/7] 启动 USB 串口通信节点...${NC}"
gnome-terminal --title="USB-Serial-Comm" -- bash -c "$SOURCE_CMD && \
    ros2 launch serial_comm serial_comm.launch.py; \
    exec bash"

sleep 2

echo -e "${GREEN}[7/7] 启动导航栈和 RViz...${NC}"
gnome-terminal --title="Navigation-Stack" -- bash -c "$SOURCE_CMD && \
    ros2 launch sentry_navigation navigation_launch.py \
    map:='$MAP_YAML' \
    use_rviz:=true; \
    exec bash"

# ===== 启动完成提示 =====
echo ""
echo -e "${GREEN}=====================================${NC}"
echo -e "${GREEN}  所有节点已在独立终端启动${NC}"
echo -e "${GREEN}=====================================${NC}"
echo ""

# ===== 系统架构说明 =====
echo -e "${YELLOW}系统架构:${NC}"
echo -e "  ${GREEN}[硬件驱动]${NC}"
echo -e "    • Livox MID360  → 点云数据发布"
echo -e ""
echo -e "  ${GREEN}[定位]${NC}"
echo -e "    • Lightning-LM   → map→odom (视觉重定位)"
echo -e "    • TF Publisher   → odom→base_link (单位变换)"
echo -e "    • Fast-LIO       → /Odometry (速度估计)"
echo -e ""
echo -e "  ${GREEN}[感知]${NC}"
echo -e "    • Livox Lidar    → 点云数据"
echo -e "    • livox_to_scan  → /scan (2D激光)"
echo -e ""
echo -e "  ${GREEN}[导航]${NC}"
echo -e "    • Nav2 Stack     → 路径规划与控制"
echo -e "    • RViz2          → 可视化界面"
echo -e ""
echo -e "  ${GREEN}[通信]${NC}"
echo -e "    • USB Serial     → 底盘控制命令"
echo -e ""

# ===== 数据流说明 =====
echo -e "${YELLOW}数据流:${NC}"
echo -e "  ${GREEN}硬件层:${NC} Livox MID360 → /livox/lidar (点云)"
echo -e "  ${GREEN}TF 树:${NC} map → odom → base_link → livox_frame"
echo -e "  ${GREEN}/odom:${NC} pose=[0,0,0], twist=Fast-LIO"
echo -e "  ${GREEN}/scan:${NC} livox_to_scan 转换的2D激光"
echo -e "  ${GREEN}/cmd_vel:${NC} Nav2 → USB Serial → 底盘电机"
echo ""

# ===== 验证命令 =====
echo -e "${YELLOW}验证命令:${NC}"
echo ""
echo -e "  ${GREEN}0. 检查 Livox 雷达数据:${NC}"
echo -e "     ros2 topic list | grep livox"
echo -e "     ros2 topic echo /livox/lidar --once"
echo -e "     ros2 topic hz /livox/lidar"
echo -e "     ${YELLOW}# 应该看到点云数据${NC}"
echo ""
echo -e "  ${GREEN}1. 查看 TF 树:${NC}"
echo -e "     ros2 run tf2_tools view_frames && evince frames.pdf"
echo ""
echo -e "  ${GREEN}2. 检查机器人位置 (从 TF):${NC}"
echo -e "     ros2 run tf2_ros tf2_echo map base_link"
echo ""
echo -e "  ${GREEN}3. 检查 /odom 话题:${NC}"
echo -e "     ros2 topic echo /odom"
echo -e "     ${YELLOW}# pose 应为 [0,0,0], twist 应有速度${NC}"
echo ""
echo -e "  ${GREEN}4. 检查速度命令:${NC}"
echo -e "     ros2 topic echo /cmd_vel"
echo -e "     ${YELLOW}# Nav2 输出的速度命令 (含 vx, vy, vtheta)${NC}"
echo ""
echo -e "  ${GREEN}5. 监控各组件频率:${NC}"
echo -e "     ros2 topic hz /livox/lidar ${YELLOW}# Livox 点云${NC}"
echo -e "     ros2 topic hz /tf          ${YELLOW}# (tf_odom_publisher)${NC}"
echo -e "     ros2 topic hz /odom        ${YELLOW}# ${NC}"
echo -e "     ros2 topic hz /Odometry    ${YELLOW}# Fast-LIO 输出${NC}"
echo -e "     ros2 topic hz /scan        ${YELLOW}# 激光扫描数据${NC}"
echo -e "     ros2 topic hz /cmd_vel     ${YELLOW}# 应约 (controller)${NC}"
echo ""
echo -e "  ${GREEN}6. 查看所有节点:${NC}"
echo -e "     ros2 node list"
echo ""
echo -e "  ${GREEN}7. 查看话题列表:${NC}"
echo -e "     ros2 topic list"
echo ""

# ===== 调试命令 =====
echo -e "${YELLOW}调试命令:${NC}"
echo ""
echo -e "  ${GREEN}检查 Livox 雷达连接:${NC}"
echo -e "     ros2 topic info /livox/lidar"
echo -e "     ${YELLOW}# 应该显示点云数据类型和发布者${NC}"
echo ""
echo -e "  ${GREEN}手动发送测试速度:${NC}"
echo -e "     ros2 topic pub /cmd_vel geometry_msgs/Twist \\"
echo -e "       '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \\"
echo -e "       -r 10"
echo -e "     ${YELLOW}# 应该看到底盘前进${NC}"
echo ""
echo -e "  ${GREEN}测试全向轮横移:${NC}"
echo -e "     ros2 topic pub /cmd_vel geometry_msgs/Twist \\"
echo -e "       '{linear: {x: 0.0, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \\"
echo -e "       -r 10"
echo -e "     ${YELLOW}# 应该看到底盘横向移动${NC}"
echo ""

echo -e "${GREEN}=====================================${NC}"
echo -e "${GREEN}  启动完成${NC}"
echo -e "${GREEN}=====================================${NC}"
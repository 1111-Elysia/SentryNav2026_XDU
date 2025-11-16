// ...existing code...
#!/bin/bash
# filepath: /home/pgd/SentryNav2026_XDU/test1.sh
# 启动流程脚本：启动节点、等待 /scan、启动导航并一次性检查 /local_costmap/costmap（超时重启脚本）

# ===== 配置区域 =====
MAP_YAML="./data/new_map/map.yaml"
SCAN_WAIT=6       # 等待 /scan 的最大秒数
COSTMAP_WAIT=10   # 启动 nav 后检查 /local_costmap/costmap 的超时时间（秒）

# ===== 颜色输出 =====
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# ===== 环境变量（用于在脚本内执行 ros2 命令） =====
SOURCE_CMD="source /opt/ros/humble/setup.bash && source ./install/setup.bash && source ../ws_livox/install/setup.bash"

# ===== 检查地图文件 =====
if [ ! -f "$MAP_YAML" ]; then
    echo -e "${RED}错误: 地图文件不存在: $MAP_YAML${NC}"
    exit 1
fi

# 确保环境在当前 shell 可用（用于后续的 ros2 topic 检查）
eval "$SOURCE_CMD"

echo -e "${GREEN}=====================================${NC}"
echo -e "${GREEN}  哨兵导航系统启动脚本${NC}"
echo -e "${GREEN}=====================================${NC}"
echo -e "${YELLOW}使用地图: $MAP_YAML${NC}"
echo ""

# 用于保存由本脚本启动的终端 PID，便于在失败时关闭
PIDS=()

# 启动函数（在后台启动 gnome-terminal 并记录 PID）
launch_term() {
    local title="$1"
    local cmd="$2"
    gnome-terminal --title="$title" -- bash -c "$SOURCE_CMD && $cmd; exec bash" &
    local pid=$!
    PIDS+=("$pid")
    echo -e "${GREEN}已启动 ${title} (pid=${pid})${NC}"
}

# ===== 启动节点（按顺序） =====
launch_term "Livox-Driver" "ros2 launch livox_ros_driver2 msg_MID360_launch.py"
sleep 3

launch_term "Lightning-LM" "ros2 run lightning run_loc_online --config ./src/lightning-lm/config/default_nclt.yaml"
sleep 2

launch_term "Fast-LIO" "ros2 launch fast_lio mapping.launch.py"
sleep 2

launch_term "TF-Odom-Publisher" "ros2 run sentry_navigation tf_odom_publisher --ros-args -p publish_rate:=50.0 -p base_link_to_livox_x:=0.117 -p base_link_to_livox_y:=0.0 -p base_link_to_livox_z:=0.0"
sleep 1

# 启动 Livox-to-Scan（第 5 步），之后脚本将等待 /scan
launch_term "Livox-to-Scan" "ros2 run livox_to_scan livox_to_scan_node --ros-args --params-file install/livox_to_scan/share/livox_to_scan/config/livox_to_scan_params.yaml"

# 等待 /scan 可用（一次性，超时则重启脚本）
echo -e "${YELLOW}[等待 /scan 话题，最长 ${SCAN_WAIT}s]${NC}"
if timeout "${SCAN_WAIT}" ros2 topic echo /scan --once >/dev/null 2>&1; then
    echo -e "${GREEN}/scan 收到消息，继续启动后续节点${NC}"
else
    echo -e "${RED}/scan 在 ${SCAN_WAIT}s 内未收到任何消息，关闭由本脚本启动的窗口并重启脚本${NC}"
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
            sleep 0.15
            kill -9 "$pid" 2>/dev/null || true
            echo -e "${YELLOW}已关闭 pid=${pid}${NC}"
        fi
    done
    sleep 1
    exec "$0" "$@"
fi

sleep 1

launch_term "USB-Serial-Comm" "ros2 launch serial_comm serial_comm.launch.py"
sleep 2

launch_term "Navigation-Stack" "ros2 launch sentry_navigation navigation_launch.py map:='$MAP_YAML' use_rviz:=true"
sleep 1

# ===== 在启动 Navigation 后一次性检查 /local_costmap/costmap =====
echo -e "${YELLOW}[检查 /local_costmap/costmap，最多等待 ${COSTMAP_WAIT}s]${NC}"
if timeout "${COSTMAP_WAIT}" ros2 topic echo /local_costmap/costmap --once >/dev/null 2>&1; then
    echo -e "${GREEN}/local_costmap/costmap 有数据，启动成功${NC}"
else
    echo -e "${RED}/local_costmap/costmap 在 ${COSTMAP_WAIT}s 内未收到消息，关闭由本脚本启动的窗口并重启脚本${NC}"
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
            sleep 0.15
            kill -9 "$pid" 2>/dev/null || true
            echo -e "${YELLOW}已关闭 pid=${pid}${NC}"
        fi
    done
    sleep 1
    exec "$0" "$@"
fi

# 启动完成提示
echo ""
echo -e "${GREEN}=====================================${NC}"
echo -e "${GREEN}  所有节点已在独立终端启动，且 /local_costmap/costmap 有数据${NC}"
echo -e "${GREEN}=====================================${NC}"
echo -e "${GREEN}启动脚本结束${NC}"
```// filepath: /home/pgd/SentryNav2026_XDU/test1.sh
// ...existing code...
#!/bin/bash
# filepath: /home/pgd/SentryNav2026_XDU/test1.sh
# 启动流程脚本：启动节点、等待 /scan、启动导航并一次性检查 /local_costmap/costmap（超时重启脚本）

# ===== 配置区域 =====
MAP_YAML="./data/new_map/map.yaml"
SCAN_WAIT=6       # 等待 /scan 的最大秒数
COSTMAP_WAIT=10   # 启动 nav 后检查 /local_costmap/costmap 的超时时间（秒）

# ===== 颜色输出 =====
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# ===== 环境变量（用于在脚本内执行 ros2 命令） =====
SOURCE_CMD="source /opt/ros/humble/setup.bash && source ./install/setup.bash && source ../ws_livox/install/setup.bash"

# ===== 检查地图文件 =====
if [ ! -f "$MAP_YAML" ]; then
    echo -e "${RED}错误: 地图文件不存在: $MAP_YAML${NC}"
    exit 1
fi

# 确保环境在当前 shell 可用（用于后续的 ros2 topic 检查）
eval "$SOURCE_CMD"

echo -e "${GREEN}=====================================${NC}"
echo -e "${GREEN}  哨兵导航系统启动脚本${NC}"
echo -e "${GREEN}=====================================${NC}"
echo -e "${YELLOW}使用地图: $MAP_YAML${NC}"
echo ""

# 用于保存由本脚本启动的终端 PID，便于在失败时关闭
PIDS=()

# 启动函数（在后台启动 gnome-terminal 并记录 PID）
launch_term() {
    local title="$1"
    local cmd="$2"
    gnome-terminal --title="$title" -- bash -c "$SOURCE_CMD && $cmd; exec bash" &
    local pid=$!
    PIDS+=("$pid")
    echo -e "${GREEN}已启动 ${title} (pid=${pid})${NC}"
}

# ===== 启动节点（按顺序） =====
launch_term "Livox-Driver" "ros2 launch livox_ros_driver2 msg_MID360_launch.py"
sleep 3

launch_term "Lightning-LM" "ros2 run lightning run_loc_online --config ./src/lightning-lm/config/default_nclt.yaml"
sleep 2

launch_term "Fast-LIO" "ros2 launch fast_lio mapping.launch.py"
sleep 2

launch_term "TF-Odom-Publisher" "ros2 run sentry_navigation tf_odom_publisher --ros-args -p publish_rate:=50.0 -p base_link_to_livox_x:=0.117 -p base_link_to_livox_y:=0.0 -p base_link_to_livox_z:=0.0"
sleep 1

# 启动 Livox-to-Scan（第 5 步），之后脚本将等待 /scan
launch_term "Livox-to-Scan" "ros2 run livox_to_scan livox_to_scan_node --ros-args --params-file install/livox_to_scan/share/livox_to_scan/config/livox_to_scan_params.yaml"

# 等待 /scan 可用（一次性，超时则重启脚本）
echo -e "${YELLOW}[等待 /scan 话题，最长 ${SCAN_WAIT}s]${NC}"
if timeout "${SCAN_WAIT}" ros2 topic echo /scan --once >/dev/null 2>&1; then
    echo -e "${GREEN}/scan 收到消息，继续启动后续节点${NC}"
else
    echo -e "${RED}/scan 在 ${SCAN_WAIT}s 内未收到任何消息，关闭由本脚本启动的窗口并重启脚本${NC}"
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
            sleep 0.15
            kill -9 "$pid" 2>/dev/null || true
            echo -e "${YELLOW}已关闭 pid=${pid}${NC}"
        fi
    done
    sleep 1
    exec "$0" "$@"
fi

sleep 1

launch_term "USB-Serial-Comm" "ros2 launch serial_comm serial_comm.launch.py"
sleep 2

launch_term "Navigation-Stack" "ros2 launch sentry_navigation navigation_launch.py map:='$MAP_YAML' use_rviz:=true"
sleep 1

# ===== 在启动 Navigation 后一次性检查 /local_costmap/costmap =====
echo -e "${YELLOW}[检查 /local_costmap/costmap，最多等待 ${COSTMAP_WAIT}s]${NC}"
if timeout "${COSTMAP_WAIT}" ros2 topic echo /local_costmap/costmap --once >/dev/null 2>&1; then
    echo -e "${GREEN}/local_costmap/costmap 有数据，启动成功${NC}"
else
    echo -e "${RED}/local_costmap/costmap 在 ${COSTMAP_WAIT}s 内未收到消息，关闭由本脚本启动的窗口并重启脚本${NC}"
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
            sleep 0.15
            kill -9 "$pid" 2>/dev/null || true
            echo -e "${YELLOW}已关闭 pid=${pid}${NC}"
        fi
    done
    sleep 1
    exec "$0" "$@"
fi

# 启动完成提示
echo ""
echo -e "${GREEN}=====================================${NC}"
echo -e "${GREEN}  所有节点已在独立终端启动，且 /local_costmap/costmap 有数据${NC}"
echo -e "${GREEN}=====================================${NC}"
echo -e "${GREEN}启动脚本结束${NC}"
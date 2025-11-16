#!/bin/bash
# 启动流程脚本：启动节点、等待 /scan、启动导航并检查 /local_costmap/costmap（超时重启脚本）
# 使用 setsid 为每个终端创建独立进程组，并记录真实 PGID，以便完全关闭本脚本启动的所有窗口

# ===== 获取脚本绝对路径（用于重启） =====
SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"

# ===== PID 文件目录 =====
PID_DIR="/tmp/sentry_nav_pids"
mkdir -p "$PID_DIR"

# ===== 配置 =====
MAP_YAML="./data/new_map/map.yaml"
SCAN_WAIT=6
COSTMAP_WAIT=10

# ===== 颜色输出 =====
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# ===== ROS2 环境 =====
SOURCE_CMD="source /opt/ros/humble/setup.bash && source ./install/setup.bash && source ../ws_livox/install/setup.bash"

# ===== 关闭所有窗口（使用正确 PGID） =====
cleanup_all_windows() {
    echo -e "${YELLOW}正在关闭所有由脚本启动的窗口...${NC}"
    local found=0

    for pidfile in "$PID_DIR"/pids_*; do
        [ -e "$pidfile" ] || continue
        pgid="$(cat "$pidfile" 2>/dev/null)"
        rm -f "$pidfile"

        if [ -n "$pgid" ]; then
            if kill -0 -"${pgid}" 2>/dev/null; then
                echo -e "${YELLOW}关闭进程组 PGID=${pgid}${NC}"
                kill -TERM -"${pgid}" 2>/dev/null || true
                sleep 0.5
                kill -KILL -"${pgid}" 2>/dev/null || true
                found=1
            fi
        fi
    done

    if [ $found -eq 1 ]; then
        echo -e "${GREEN}已关闭所有相关窗口${NC}"
    else
        echo -e "${GREEN}未找到需要关闭的窗口（可能已全部关闭）${NC}"
    fi
}

# 启动时自动先清理
cleanup_all_windows

# ===== 检查地图文件 =====
if [ ! -f "$MAP_YAML" ]; then
    echo -e "${RED}错误: 地图文件不存在: $MAP_YAML${NC}"
    exit 1
fi

eval "$SOURCE_CMD"

echo -e "${GREEN}=====================================${NC}"
echo -e "${GREEN}  哨兵导航系统启动脚本${NC}"
echo -e "${GREEN}=====================================${NC}"
echo -e "${YELLOW}使用地图: $MAP_YAML${NC}"
echo ""

# ===== 正确版本的终端启动函数（完全修复 PGID 记录问题） =====
launch_term() {
    local title="$1"
    local cmd="$2"

    # setsid 直接执行 gnome-terminal（无中间 shell）
    setsid gnome-terminal --title="SentryNav-${title}" \
        -- bash -c "$SOURCE_CMD && $cmd; exec bash" >/dev/null 2>&1 &

    local term_pid=$!  # <--- 关键：这里就是 gnome-terminal 的 PID
    sleep 0.1

    # 获取真实 PGID（gnome-terminal 进程组）
    local pgid
    pgid=$(ps -o pgid= -p "$term_pid" | tr -d ' ')

    if [ -n "$pgid" ]; then
        echo "$pgid" > "$PID_DIR/pids_${pgid}"
        echo -e "${GREEN}已启动 ${title} (pid=${term_pid}, pgid=${pgid})${NC}"
        return 0
    else
        echo -e "${RED}启动 ${title} 失败：无法获取 PGID${NC}"
        return 1
    fi
}

# ===== 启动节点（按顺序） =====
launch_term "Livox-Driver" "ros2 launch livox_ros_driver2 msg_MID360_launch.py"
sleep 3

launch_term "Lightning-LM" "ros2 run lightning run_loc_online --config ./src/lightning-lm/config/default_nclt.yaml"
sleep 2

launch_term "Fast-LIO" "ros2 launch fast_lio mapping.launch.py"
sleep 2

launch_term "TF-Odom-Publisher" "ros2 run sentry_navigation tf_odom_publisher --ros-args --params-file ./src/sentry_navigation/config/lidar.yaml"
sleep 1

launch_term "Livox-to-Scan" "ros2 run livox_to_scan livox_to_scan_node --ros-args --params-file install/livox_to_scan/share/livox_to_scan/config/livox_to_scan_params.yaml"

# ===== 等待 /scan =====
echo -e "${YELLOW}[等待 /scan，最多 ${SCAN_WAIT}s]${NC}"
if timeout "$SCAN_WAIT" ros2 topic echo /scan --once >/dev/null 2>&1; then
    echo -e "${GREEN}/scan 收到消息${NC}"
else
    echo -e "${RED}/scan 超时，准备重启脚本${NC}"
    cleanup_all_windows
    sleep 1
    exec bash "$SCRIPT_PATH"
fi

sleep 1

launch_term "USB-Serial-Comm" "ros2 launch serial_comm serial_comm.launch.py"
sleep 2

launch_term "Navigation-Stack" "ros2 launch sentry_navigation navigation_launch.py map:='$MAP_YAML' use_rviz:=true"
sleep 1

# ===== 检查 costmap =====
echo -e "${YELLOW}[检查 /local_costmap/costmap，最多 ${COSTMAP_WAIT}s]${NC}"
if timeout "$COSTMAP_WAIT" ros2 topic echo /local_costmap/costmap --once >/dev/null 2>&1; then
    echo -e "${GREEN}/local_costmap/costmap 正常${NC}"
else
    echo -e "${RED}/local_costmap/costmap 超时，准备重启脚本${NC}"
    cleanup_all_windows
    sleep 1
    exec bash "$SCRIPT_PATH"
fi

echo ""
echo -e "${GREEN}=====================================${NC}"
echo -e "${GREEN}  所有节点已在独立终端启动${NC}"
echo -e "${GREEN}=====================================${NC}"
echo -e "${GREEN}启动脚本结束${NC}"

#!/bin/bash
# 全新修复版：稳定启动 /scan 等待 / 开启导航 / 关闭所有窗口（PGID 正确）

# ===== 获取脚本绝对路径 =====
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

# ============================================================
#   完整修复版窗口关闭函数（保证所有窗口都能关闭）
# ============================================================
cleanup_all_windows() {
    echo -e "${YELLOW}正在关闭所有由脚本启动的窗口...${NC}"

    local found=0

    # 关闭记录的 PGID
    for pidfile in "$PID_DIR"/pids_*; do
        [ -e "$pidfile" ] || continue
        pgid="$(cat "$pidfile" 2>/dev/null)"
        rm -f "$pidfile"

        if [ -n "$pgid" ]; then
            if kill -0 -"${pgid}" 2>/dev/null; then
                echo -e "${YELLOW}关闭 PGID=${pgid}${NC}"
                kill -TERM -"${pgid}" 2>/dev/null || true
                sleep 0.4
                kill -KILL -"${pgid}" 2>/dev/null || true
                found=1
            fi
        fi
    done

    # fallback：确保杀掉没记录到 PGID 的终端
    pkill -f "SentryNav-" 2>/dev/null && found=1

    if [ $found -eq 1 ]; then
        echo -e "${GREEN}所有窗口均已关闭${NC}"
    else
        echo -e "${GREEN}无可关闭窗口${NC}"
    fi
}

# 脚本启动前先清理
cleanup_all_windows

# ===== 检查地图 =====
if [ ! -f "$MAP_YAML" ]; then
    echo -e "${RED}错误: 地图文件不存在: $MAP_YAML${NC}"
    exit 1
fi

eval "$SOURCE_CMD"

echo -e "${GREEN}====== 哨兵导航系统启动 ======${NC}"
echo -e "${YELLOW}使用地图: $MAP_YAML${NC}"

# ============================================================
#   稳定终端启动函数（关键：使用 nohup + gnome-terminal）
# ============================================================
launch_term() {
    local title="$1"
    local cmd="$2"

    nohup gnome-terminal --title="SentryNav-${title}" \
        -- bash -c "$SOURCE_CMD && $cmd; exec bash" >/dev/null 2>&1 &

    local term_pid=$!
    sleep 0.15

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

# ============================================================
#   安全等待话题（ros2 topic echo 不会等待）
# ============================================================
wait_topic() {
    local topic="$1"
    local timeout="$2"

    echo -e "${YELLOW}[等待 ${topic}，最多 ${timeout}s]${NC}"
    local elapsed=0

    while [ $elapsed -lt $timeout ]; do
        if ros2 topic list | grep -q "^${topic}$"; then
            echo -e "${GREEN}${topic} 已出现${NC}"
            return 0
        fi
        sleep 0.2
        elapsed=$((elapsed + 1))
    done

    echo -e "${RED}${topic} 超时${NC}"
    return 1
}

# ============================================================
#   顺序启动节点
# ============================================================
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
if ! wait_topic "/scan" "$SCAN_WAIT"; then
    cleanup_all_windows
    exec bash "$SCRIPT_PATH"
fi

sleep 1

launch_term "USB-Serial-Comm" "ros2 launch serial_comm serial_comm.launch.py"
sleep 2

launch_term "Navigation-Stack" "ros2 launch sentry_navigation navigation_launch.py map:='$MAP_YAML' use_rviz:=true"
sleep 1

# ===== 等待 costmap =====
if ! wait_topic "/local_costmap/costmap" "$COSTMAP_WAIT"; then
    cleanup_all_windows
    exec bash "$SCRIPT_PATH"
fi

echo -e "${GREEN}====== 所有节点已成功启动 ======${NC}"

#!/bin/bash

# ========================
#   基础配置
# ========================
SOURCE_CMD="source /opt/ros/humble/setup.bash && source ./install/setup.bash && source ../ws_livox/install/setup.bash"
MAP_YAML="./data/new_map/map.yaml"
SCAN_WAIT=6
COSTMAP_WAIT=10

PID_DIR="/tmp/sentry_nav_pids"
TITLE_PREFIX="SentryNav"
mkdir -p "$PID_DIR"

# ========================
# 颜色
# ========================
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'


# ========================
#  关闭所有窗口（按 PGID）
# ========================
cleanup_all_windows() {
    echo -e "${YELLOW}正在关闭所有启动的窗口...${NC}"

    for pf in "$PID_DIR"/pids_*; do
        [ -e "$pf" ] || continue

        pgid=$(cat "$pf")
        echo -e "${YELLOW}关闭 PGID=${pgid}${NC}"

        kill -TERM -"$pgid" 2>/dev/null
        sleep 0.5
        kill -KILL -"$pgid" 2>/dev/null

        rm -f "$pf"
    done

    echo -e "${GREEN}窗口关闭完成${NC}"
}


# ========================
#  启动终端（最终稳定版）
# ========================
launch_term() {
    local title="$1"
    local cmd="$2"
    local win_title="${TITLE_PREFIX}-${title}"

    echo -e "${GREEN}启动窗口: ${title}${NC}"

    gnome-terminal \
        --title="$win_title" \
        -- bash -ic "$SOURCE_CMD; $cmd; exec bash" &

    sleep 0.5

    # 找到这个窗口对应的进程
    pid=$(pgrep -f "$win_title" | head -n 1)
    if [ -z "$pid" ]; then
        echo -e "${RED}[ERROR] 启动 ${title} 失败：找不到 PID${NC}"
        return 1
    fi

    pgid=$(ps -o pgid= -p "$pid" | tr -d ' ')
    if [ -z "$pgid" ]; then
        echo -e "${RED}[ERROR] 找不到 PGID${NC}"
        return 1
    fi

    echo "$pgid" > "$PID_DIR/pids_${pgid}"
    echo -e "${GREEN}已启动 ${title} (pid=$pid, pgid=$pgid)${NC}"
}


# ========================
#  等待 /scan
# ========================
wait_for_scan() {
    echo -e "${YELLOW}[等待 /scan 话题，最长 ${SCAN_WAIT}s]${NC}"

    if timeout $SCAN_WAIT bash -c '
        while ! ros2 topic list | grep -q "^/scan$"; do
            sleep 0.2
        done
    '; then
        echo -e "${GREEN}/scan 已出现${NC}"
        return 0
    fi

    echo -e "${RED}/scan 未在 ${SCAN_WAIT}s 内出现${NC}"
    return 1
}


# ========================
#  启动流程
# ========================
echo -e "${GREEN}=====================================${NC}"
echo -e "${GREEN}  哨兵导航系统启动脚本${NC}"
echo -e "${GREEN}=====================================${NC}"

cleanup_all_windows

eval "$SOURCE_CMD"

# 启动节点顺序（按你的原脚本）
launch_term "Livox-Driver" "ros2 launch livox_ros_driver2 msg_MID360_launch.py"
sleep 3

launch_term "Lightning-LM" "ros2 run lightning run_loc_online --config ./src/lightning-lm/config/default_nclt.yaml"
sleep 2

launch_term "Fast-LIO" "ros2 launch fast_lio mapping.launch.py"
sleep 2

launch_term "TF-Odom-Publisher" "ros2 run sentry_navigation tf_odom_publisher --ros-args --params-file ./src/sentry_navigation/config/lidar.yaml"
sleep 1

launch_term "Livox-to-Scan" "ros2 run livox_to_scan livox_to_scan_node --ros-args --params-file install/livox_to_scan/share/livox_to_scan/config/livox_to_scan_params.yaml"

# 等待 /scan
if ! wait_for_scan; then
    echo -e "${RED}重启脚本...${NC}"
    cleanup_all_windows
    exec bash "$0" "$@"
fi

sleep 1

launch_term "USB-Serial-Comm" "ros2 launch serial_comm serial_comm.launch.py"
sleep 2

launch_term "Navigation-Stack" "ros2 launch sentry_navigation navigation_launch.py map:='$MAP_YAML' use_rviz:=true"

echo -e "${YELLOW}[检查 /local_costmap/costmap，最长 ${COSTMAP_WAIT}s]${NC}"
if timeout $COSTMAP_WAIT ros2 topic echo /local_costmap/costmap --once >/dev/null 2>&1; then
    echo -e "${GREEN}/local_costmap/costmap 有数据，启动成功${NC}"
else
    echo -e "${RED}/local_costmap/costmap 超时，重启脚本${NC}"
    cleanup_all_windows
    exec bash "$0" "$@"
fi

echo -e "${GREEN}=====================================${NC}"
echo -e "${GREEN}  所有节点已成功启动${NC}"
echo -e "${GREEN}=====================================${NC}"

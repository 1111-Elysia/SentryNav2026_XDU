#!/bin/bash
# 启动流程脚本：启动节点、等待 /scan、启动导航并一次性检查 /local_costmap/costmap（超时重启脚本）
# 使用 setsid 为每个终端创建独立进程组，记录 PGID 与组内 PID，以便彻底关闭本脚本启动的所有窗口

# ===== 获取脚本绝对路径（用于重启） =====
SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"

# ===== PID 文件目录（记录所有由本脚本启动的进程组和 PID 列表） =====
PID_DIR="/tmp/sentry_nav_pids"
mkdir -p "$PID_DIR"

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

# ===== 函数：关闭所有由本脚本记录的进程组（彻底关闭） =====
cleanup_all_windows() {
    echo -e "${YELLOW}正在关闭所有由本脚本启动的窗口...${NC}"
    local any_found=0

    for pidfile in "$PID_DIR"/pids_*; do
        [ -e "$pidfile" ] || continue

        # 文件第一行为 PGID，后续为启动时记录的 PID 列表
        pgid="$(sed -n '1p' "$pidfile" 2>/dev/null | tr -d ' ')"
        pids_in_file="$(sed -n '2,$p' "$pidfile" 2>/dev/null | tr -s '\n' ' ')"

        # 删除记录文件（避免重复处理）
        rm -f "$pidfile"

        if [ -n "$pgid" ]; then
            any_found=1
            echo -e "${YELLOW}尝试关闭进程组 PGID=${pgid}（按组终止）${NC}"
            # 优雅终止进程组
            pkill -TERM -g "$pgid" 2>/dev/null || true
            sleep 0.25
            # 强制终止进程组
            pkill -KILL -g "$pgid" 2>/dev/null || true
            sleep 0.05

            # 逐个确认并杀掉记录的 PID（保险）
            for pid in $pids_in_file; do
                pid="$(echo "$pid" | tr -d ' ')"
                if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
                    echo -e "${YELLOW}强制关闭 pid=${pid}${NC}"
                    kill -TERM "$pid" 2>/dev/null || true
                    sleep 0.05
                    kill -KILL "$pid" 2>/dev/null || true
                fi
            done

            # 最后，确保组内没有残留进程
            for leftover in $(ps -o pid= --pgid "$pgid" 2>/dev/null || true); do
                leftover="$(echo "$leftover" | tr -d ' ')"
                if [ -n "$leftover" ]; then
                    echo -e "${YELLOW}清理残留 pid=${leftover}${NC}"
                    kill -TERM "$leftover" 2>/dev/null || true
                    sleep 0.05
                    kill -KILL "$leftover" 2>/dev/null || true
                fi
            done
        fi
    done

    if [ $any_found -eq 1 ]; then
        sleep 0.5
        echo -e "${GREEN}已尝试关闭所有相关窗口${NC}"
    else
        echo -e "${GREEN}未找到需要关闭的窗口（或已全部关闭）${NC}"
    fi
}

# 在脚本开始时先清理旧的记录（尝试关闭遗留进程组）
cleanup_all_windows

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

# ===== 启动函数：用 setsid 启动 gnome-terminal（创建新进程组），并记录 PGID 与组内 PIDs 到文件 =====
launch_term() {
    local title="$1"
    local cmd="$2"

    # 使用 setsid 启动新的会话并在后台运行
    setsid gnome-terminal --title="SentryNav-${title}" -- bash -c "$SOURCE_CMD && $cmd; exec bash" >/dev/null 2>&1 &
    local child_pid=$!

    # 等待短时间以确保进程已启动并组已建立
    sleep 0.12

    # 获取进程组 ID（PGID）
    pgid="$(ps -o pgid= -p "$child_pid" 2>/dev/null | tr -d ' ')"
    if [ -z "$pgid" ]; then
        # 回退：如果无法得到 PGID，就尝试用 child pid 作为标识
        pgid="$child_pid"
    fi

    # 记录当前进程组内的 PID 列表以便后续强力清理
    pids="$(ps -o pid= --pgid "$pgid" 2>/dev/null || echo "$child_pid")"
    pidfile="$PID_DIR/pids_${pgid}_$(date +%s%N)"
    {
        echo "$pgid"
        for pid in $pids; do
            echo "$pid"
        done
    } > "$pidfile"

    echo -e "${GREEN}已启动 ${title} (starter_pid=${child_pid}, pgid=${pgid})，记录: $pidfile${NC}"
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

# 启动 Livox-to-Scan（第 5 步），之后脚本将等待 /scan
launch_term "Livox-to-Scan" "ros2 run livox_to_scan livox_to_scan_node --ros-args --params-file install/livox_to_scan/share/livox_to_scan/config/livox_to_scan_params.yaml"

# 等待 /scan 可用（一次性，超时则重启脚本）
echo -e "${YELLOW}[等待 /scan 话题，最长 ${SCAN_WAIT}s]${NC}"
if timeout "${SCAN_WAIT}" ros2 topic echo /scan --once >/dev/null 2>&1; then
    echo -e "${GREEN}/scan 收到消息，继续启动后续节点${NC}"
else
    echo -e "${RED}/scan 在 ${SCAN_WAIT}s 内未收到任何消息，准备重启脚本${NC}"
    cleanup_all_windows
    sleep 1
    exec bash "$SCRIPT_PATH" "$@"
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
    echo -e "${RED}/local_costmap/costmap 在 ${COSTMAP_WAIT}s 内未收到消息，准备重启脚本${NC}"
    cleanup_all_windows
    sleep 1
    exec bash "$SCRIPT_PATH" "$@"
fi

# 启动完成提示
echo ""
echo -e "${GREEN}=====================================${NC}"
echo -e "${GREEN}  所有节点已在独立终端启动，且 /local_costmap/costmap 有数据${NC}"
echo -e "${GREEN}=====================================${NC}"
echo -e "${GREEN}启动脚本结束${NC}"
echo
```// filepath: /home/pgd/SentryNav2026_XDU/test1.sh
#!/bin/bash
# 启动流程脚本：启动节点、等待 /scan、启动导航并一次性检查 /local_costmap/costmap（超时重启脚本）
# 使用 setsid 为每个终端创建独立进程组，记录 PGID 与组内 PID，以便彻底关闭本脚本启动的所有窗口

# ===== 获取脚本绝对路径（用于重启） =====
SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"

# ===== PID 文件目录（记录所有由本脚本启动的进程组和 PID 列表） =====
PID_DIR="/tmp/sentry_nav_pids"
mkdir -p "$PID_DIR"

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

# ===== 函数：关闭所有由本脚本记录的进程组（彻底关闭） =====
cleanup_all_windows() {
    echo -e "${YELLOW}正在关闭所有由本脚本启动的窗口...${NC}"
    local any_found=0

    for pidfile in "$PID_DIR"/pids_*; do
        [ -e "$pidfile" ] || continue

        # 文件第一行为 PGID，后续为启动时记录的 PID 列表
        pgid="$(sed -n '1p' "$pidfile" 2>/dev/null | tr -d ' ')"
        pids_in_file="$(sed -n '2,$p' "$pidfile" 2>/dev/null | tr -s '\n' ' ')"

        # 删除记录文件（避免重复处理）
        rm -f "$pidfile"

        if [ -n "$pgid" ]; then
            any_found=1
            echo -e "${YELLOW}尝试关闭进程组 PGID=${pgid}（按组终止）${NC}"
            # 优雅终止进程组
            pkill -TERM -g "$pgid" 2>/dev/null || true
            sleep 0.25
            # 强制终止进程组
            pkill -KILL -g "$pgid" 2>/dev/null || true
            sleep 0.05

            # 逐个确认并杀掉记录的 PID（保险）
            for pid in $pids_in_file; do
                pid="$(echo "$pid" | tr -d ' ')"
                if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
                    echo -e "${YELLOW}强制关闭 pid=${pid}${NC}"
                    kill -TERM "$pid" 2>/dev/null || true
                    sleep 0.05
                    kill -KILL "$pid" 2>/dev/null || true
                fi
            done

            # 最后，确保组内没有残留进程
            for leftover in $(ps -o pid= --pgid "$pgid" 2>/dev/null || true); do
                leftover="$(echo "$leftover" | tr -d ' ')"
                if [ -n "$leftover" ]; then
                    echo -e "${YELLOW}清理残留 pid=${leftover}${NC}"
                    kill -TERM "$leftover" 2>/dev/null || true
                    sleep 0.05
                    kill -KILL "$leftover" 2>/dev/null || true
                fi
            done
        fi
    done

    if [ $any_found -eq 1 ]; then
        sleep 0.5
        echo -e "${GREEN}已尝试关闭所有相关窗口${NC}"
    else
        echo -e "${GREEN}未找到需要关闭的窗口（或已全部关闭）${NC}"
    fi
}

# 在脚本开始时先清理旧的记录（尝试关闭遗留进程组）
cleanup_all_windows

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

# ===== 启动函数：用 setsid 启动 gnome-terminal（创建新进程组），并记录 PGID 与组内 PIDs 到文件 =====
launch_term() {
    local title="$1"
    local cmd="$2"

    # 使用 setsid 启动新的会话并在后台运行
    setsid gnome-terminal --title="SentryNav-${title}" -- bash -c "$SOURCE_CMD && $cmd; exec bash" >/dev/null 2>&1 &
    local child_pid=$!

    # 等待短时间以确保进程已启动并组已建立
    sleep 0.12

    # 获取进程组 ID（PGID）
    pgid="$(ps -o pgid= -p "$child_pid" 2>/dev/null | tr -d ' ')"
    if [ -z "$pgid" ]; then
        # 回退：如果无法得到 PGID，就尝试用 child pid 作为标识
        pgid="$child_pid"
    fi

    # 记录当前进程组内的 PID 列表以便后续强力清理
    pids="$(ps -o pid= --pgid "$pgid" 2>/dev/null || echo "$child_pid")"
    pidfile="$PID_DIR/pids_${pgid}_$(date +%s%N)"
    {
        echo "$pgid"
        for pid in $pids; do
            echo "$pid"
        done
    } > "$pidfile"

    echo -e "${GREEN}已启动 ${title} (starter_pid=${child_pid}, pgid=${pgid})，记录: $pidfile${NC}"
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

# 启动 Livox-to-Scan（第 5 步），之后脚本将等待 /scan
launch_term "Livox-to-Scan" "ros2 run livox_to_scan livox_to_scan_node --ros-args --params-file install/livox_to_scan/share/livox_to_scan/config/livox_to_scan_params.yaml"

# 等待 /scan 可用（一次性，超时则重启脚本）
echo -e "${YELLOW}[等待 /scan 话题，最长 ${SCAN_WAIT}s]${NC}"
if timeout "${SCAN_WAIT}" ros2 topic echo /scan --once >/dev/null 2>&1; then
    echo -e "${GREEN}/scan 收到消息，继续启动后续节点${NC}"
else
    echo -e "${RED}/scan 在 ${SCAN_WAIT}s 内未收到任何消息，准备重启脚本${NC}"
    cleanup_all_windows
    sleep 1
    exec bash "$SCRIPT_PATH" "$@"
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
    echo -e "${RED}/local_costmap/costmap 在 ${COSTMAP_WAIT}s 内未收到消息，准备重启脚本${NC}"
    cleanup_all_windows
    sleep 1
    exec bash "$SCRIPT_PATH" "$@"
fi

# 启动完成提示
echo ""
echo -e "${GREEN}=====================================${NC}"
echo -e "${GREEN}  所有节点已在独立终端启动，且 /local_costmap/costmap 有数据${NC}"
echo -e "${GREEN}=====================================${NC}"
echo -e "${GREEN}启动脚本结束${NC}"
echo
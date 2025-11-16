#!/bin/bash

# ========================
#   参数配置
# ========================

SOURCE_CMD="source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash"
PID_DIR="/tmp/sentry_pids"
SCAN_WAIT=6  # 等待 /scan 的秒数
KILL_WAIT=1  # 关闭窗口后的延迟
TITLE_PREFIX="SentryNav"  # 终端标题前缀

mkdir -p "$PID_DIR"


# ========================
#   启动终端函数（最终版）
# ========================
launch_term() {
    local title="$1"
    local cmd="$2"
    local win_title="${TITLE_PREFIX}-${title}"

    echo -e "[INFO] 启动窗口: ${win_title}"

    # 启动终端（不依赖 setsid）
    nohup gnome-terminal --title="${win_title}" \
        -- bash -c "$SOURCE_CMD && $cmd; exec bash" >/dev/null 2>&1 &

    # 等待终端 fork 完成
    sleep 0.6

    # 查找带窗口标题的终端 PID（最稳方案）
    local pid
    pid=$(pgrep -f "${win_title}" | head -n 1)

    if [ -z "$pid" ]; then
        echo -e "[ERROR] 未找到终端进程: ${win_title}"
        return 1
    fi

    # 获取 PGID
    local pgid
    pgid=$(ps -o pgid= -p "$pid" | tr -d ' ')

    if [ -n "$pgid" ]; then
        echo "$pgid" > "${PID_DIR}/pids_${pgid}"
        echo -e "[INFO] 成功启动: ${win_title} (pid=$pid, pgid=$pgid)"
    else
        echo -e "[ERROR] 启动 ${win_title} 但无法获取 PGID"
    fi
}


# ========================
#   关闭所有窗口
# ========================
cleanup_all_windows() {
    echo -e "[INFO] 正在关闭所有窗口..."

    for f in "$PID_DIR"/pids_*; do
        [ -e "$f" ] || continue

        pgid=$(basename "$f" | cut -d_ -f2)

        echo -e "[INFO] kill PGID: $pgid"

        # 关闭整个进程组
        kill -TERM -"$pgid" 2>/dev/null
        sleep 0.3
        kill -KILL -"$pgid" 2>/dev/null

        rm -f "$f"
    done

    sleep "$KILL_WAIT"
    echo -e "[INFO] 窗口已关闭"
}


# ========================
#   等待 /scan 稳定出现
# ========================
wait_for_scan() {
    echo -e "[INFO] 等待 /scan 出现..."

    if timeout "$SCAN_WAIT" bash -c '
        while ! ros2 topic list | grep -q "^/scan$"; do
            sleep 0.1
        done
    '; then
        echo -e "[INFO] 检测到 /scan"
        return 0
    else
        echo -e "[WARN] /scan 超时 ${SCAN_WAIT}s"
        return 1
    fi
}


# ========================
#   启动所有节点
# ========================
start_all_nodes() {

    cleanup_all_windows  # 先清空旧窗口

    launch_term "laser_lvx2bag" "ros2 launch rose_simulator laser_lvx_to_bag.launch.py"

    launch_term "laser2" "ros2 launch rose_robot rose_livox.launch.py"

    launch_term "mapping" "ros2 launch rose_robot rose_mapping_nocollision.launch.py"

    launch_term "navigation" "ros2 launch rose_robot rose_global_planning_nav.launch.py rviz:=false"

    launch_term "camera" "ros2 launch rose_robot rose_rgbd_camera.launch.py"

    launch_term "behavior" "ros2 launch test_behavior behavior_navigation.launch.py"

    launch_term "local_mask" "ros2 launch test_behavior mask_avoidance.launch.py"

    launch_term "global_remap" "ros2 run test_behavior global_remap"

    launch_term "rotate" "ros2 run laser_rotate rotate"

    launch_term "print" "ros2 run print_test print_node"
}


# ========================
#   主循环（可放 systemd）
# ========================
echo -e "==============================="
echo -e "  自动启动与监测脚本开始运行"
echo -e "===============================\n"

while true; do

    start_all_nodes

    # 等待 /scan 出现
    wait_for_scan
    if [ $? -ne 0 ]; then
        echo -e "[ERROR] /scan 未出现 → 重启所有节点"
        continue    # 回到 start_all_nodes
    fi

    echo -e "[INFO] 系统已启动，进入监控模式"

    # === 根据需要可以添加监控逻辑 ===
    # 比如检测节点是否退出
    # detect_failure || continue

    # 暂时只保持运行
    sleep 999999

done

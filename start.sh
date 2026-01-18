#!/bin/bash
source ./install/setup.bash
echo "导航，启动！"

# 捕获 Ctrl+C，直接退出脚本
trap 'echo "[INFO] Ctrl+C 被按下，退出脚本"; exit 0' SIGINT

while true; do
    echo

    # ros2 launch 输出通过 grep 过滤指定节点日志
    ros2 launch bringup monitored_start.launch.py 2>&1 | \
    grep -v -E "\[(controller_server|planner_server|rviz2|global_costmap|local_costmap)\].*"

    # 捕获 ros2 launch 的退出码（管道第一个命令）
    EXIT_CODE=${PIPESTATUS[0]}

    echo "[INFO] Launch 退出，exit code=$EXIT_CODE"

    # 非 0 异常退出 → 重启
    if [ "$EXIT_CODE" -ne 0 ]; then
        echo "[ERROR] 导航异常退出，自动重启中..."
        sleep 1
    else
        # 正常退出，退出循环
        break
    fi
done

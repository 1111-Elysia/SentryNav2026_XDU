#!/bin/bash
source ./install/setup.bash
echo "导航，启动！"

# 捕获 Ctrl+C，直接退出脚本
trap 'echo "[INFO] Ctrl+C 被按下，退出脚本"; exit 0' SIGINT

while true; do
    echo
    
    # 直接运行 launch
    ros2 launch bringup monitored_start.launch.py
    EXIT_CODE=$?

    echo "[INFO] Launch 退出，exit code=$EXIT_CODE"

    # Ctrl+C 触发的退出由 trap 捕获，脚本直接退出

    # 非 0 异常退出 → 重启
    if [ "$EXIT_CODE" -ne 0 ]; then
        echo "[ERROR] 导航异常退出，自动重启中..."
        sleep 1
    else
        # 正常退出（0）也退出循环
        break
    fi
done

#!/bin/bash

# ==============================================================================
# 全局清理函数 
# 作用：无论进程在哪个终端运行，只要匹配名字，全部强制杀死 (SIGKILL -9)
# ==============================================================================
function nuclear_cleanup() {
    # 防止信号递归：清理期间忽略 SIGINT/SIGTERM
    trap '' SIGINT SIGTERM

    echo "---------------------------------------------------------"
    echo "[SYSTEM] 正在执行全系统强力清理 (Global Kill)..."

    # 1. 杀掉所有 ROS 2 节点 (通过 ros2 daemon 获取完整列表)
    # 先尝试优雅 shutdown lifecycle 节点（带超时保护，防止 daemon 异常时卡死）
    for node in $(timeout 5 ros2 node list 2>/dev/null); do
        timeout 3 ros2 lifecycle set "$node" shutdown 2>/dev/null
    done

    # 2. 杀掉 Python 节点 (pkill -f 匹配命令行)
    pkill -9 -f "tf_monitor.py"
    pkill -9 -f "ground_pos_simulator.py"

    # 3. 杀掉 感知与SLAM
    killall -9 livox_ros_driver2_node super_lio_Mapping \
               lidar_filter_node depth_to_pcl_node \
               livox_to_scan_node 2>/dev/null

    # 4. 杀掉 TF 与里程计
    killall -9 tf_only_odom tf_odom_publisher odin_tf 2>/dev/null

    # 5. 杀掉 通信层 (CAN + 串口)
    killall -9 can_comm_node can_receive_node \
               target_frame_node yaw_controller_node \
               serial_comm_node 2>/dev/null

    # 6. 杀掉 裁判系统与位置中继
    killall -9 ground_pos_relay_node ground_pos_relay_sim_node \
               teammate_frame_converter_node 2>/dev/null

    # 7. 杀掉 行为树与决策
    killall -9 sentry_nav_bt_test 2>/dev/null

    # 8. 杀掉 Nav2 栈 (lifecycle 节点)
    killall -9 lifecycle_manager map_server map_saver \
               controller_server planner_server \
               behavior_server smoother_server velocity_smoother \
               bt_navigator waypoint_follower 2>/dev/null

    # 9. 杀掉 可视化与辅助工具
    killall -9 rviz2 robot_state_publisher \
               static_transform_publisher 2>/dev/null

    # 10. 兜底：按包名 pkill 残留进程
    pkill -9 -f "rm_referee"
    pkill -9 -f "sentry_nav"
    pkill -9 -f "bringup"
    pkill -9 -f "can_comm"
    pkill -9 -f "cpp_lidar"
    pkill -9 -f "super_lio"
    pkill -9 -f "ground_pos_relay"

    # 11. 注意：不杀 ROS 2 daemon，ros2 launch 需要它
    # ros2 daemon stop 会让后续 launch 卡死，残留 daemon 由 ros2 自行管理
    # 如果确实需要重置 daemon，手动执行: ros2 daemon stop && ros2 daemon start

    echo "[SYSTEM] 再见，愿哨兵小姐一路顺风..."
    echo "---------------------------------------------------------"
}

# ==============================================================================
# 主逻辑
# ==============================================================================

# ==============================================================================
# 脚本自定位：确保无论从哪里调用，都以此为工作目录
# ==============================================================================
SCRIPT_DIR="/home/nuc/Desktop/SentryNav2026_XDU"
cd "$SCRIPT_DIR" || exit 1

# 源环境（强制使用本目录下的 install）
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
else
    echo "[FATAL] 找不到 $SCRIPT_DIR/install/setup.bash，请先 colcon build"
    exit 1
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "导航系统主控脚本启动... (工作目录: $SCRIPT_DIR)"

# 捕获终止信号（Ctrl+C / systemctl stop / kill 均生效）
trap 'echo "[INFO] 收到终止信号，正在清理..."; nuclear_cleanup; exit 0' SIGINT SIGTERM

while true; do
    echo "=========================================="
    echo ">>> 赞美欧姆弥赛亚，愿哨兵小姐武运昌隆"
    
    # 每次启动前，先执行核弹以保平安
    # nuclear_cleanup
    # 恢复信号捕获（nuclear_cleanup 内部屏蔽了信号，这里重新启用）
    trap 'echo "[INFO] 收到终止信号，正在清理..."; nuclear_cleanup; exit 0' SIGINT SIGTERM

    # 稍微等待进程完全释放
    sleep 1

    echo ">>> 启动 Launch 文件..."
    # ---------------------------------------------------------
    # 启动命令
    # 2>&1 | grep ... 用于过滤掉不想看的刷屏日志
    # ---------------------------------------------------------
    ros2 launch bringup start.launch.py 2>&1 | \
    grep -v -E "\[(rviz2|controller_server|planner_server|global_costmap|local_costmap)\].*(Message Filter dropping message|Robot is out of bounds)"

    # 获取 ros2 launch 的退出码
    EXIT_CODE=${PIPESTATUS[0]}

    echo ">>> Launch 进程结束，退出码: $EXIT_CODE"

    # 如果是正常关闭 (Ctrl+C 传递下来的) -> 退出循环
    # 如果是异常崩溃 (exit code != 0) -> 重启
    if [ "$EXIT_CODE" -eq 0 ]; then
        echo "[INFO] 正常退出。"
        nuclear_cleanup
        break
    else
        echo "[WARN] 检测到异常退出/监控触发重启，2秒后重新拉起..."
        sleep 2
    fi
done
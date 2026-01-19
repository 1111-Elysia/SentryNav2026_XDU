#!/bin/bash

# ==============================================================================
# 全局清理函数 - 核弹级
# 作用：无论进程在哪个终端运行，只要匹配名字，全部强制杀死 (SIGKILL -9)
# ==============================================================================
function nuclear_cleanup() {
    echo "---------------------------------------------------------"
    echo "[SYSTEM] 正在执行全系统强力清理 (Global Kill)..."
    
    # 1. 杀掉 Python 监控与管理节点 (匹配完整命令行)
    # pkill -f 会匹配整个命令行参数，不仅是进程名
    pkill -9 -f "tf_monitor.py"
    pkill -9 -f "scan_monitor.py"
    pkill -9 -f "costmap_monitor.py"
    pkill -9 -f "map_monitor.py"
    pkill -9 -f "nav_loc_monitor.py"
    pkill -9 -f "restart_manager.py"
    
    # 2. 杀掉 Nav2 相关的 Lifecycle 节点和服务器
    # killall 匹配进程可执行文件名
    killall -9 nav2_lifecycle_manager lifecycle_manager map_server \
               controller_server planner_server recoveries_server \
               behavior_server smoother_server velocity_smoother \
               bt_navigator waypoint_follower amcl 2>/dev/null

    # 3. 杀掉 核心算法与驱动
    killall -9 fast_lio_multi livox_ros_driver2_node serial_comm_node \
               kiss_icp_node ekf_node point_cloud_transport 2>/dev/null

    # 4. 杀掉 可视化与辅助工具
    killall -9 rviz2 static_transform_publisher robot_state_publisher \
               joint_state_publisher component_container component_container_isolated 2>/dev/null

    # 5. 清理 ROS 2 守护进程 (可选，有时候 daemon 会缓存旧参数)
    # ros2 daemon stop
    
    echo "[SYSTEM] 清理完毕，内存已释放。"
    echo "---------------------------------------------------------"
}

# ==============================================================================
# 主逻辑
# ==============================================================================

# 源环境
source ./install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "导航系统主控脚本启动..."

# 捕获 Ctrl+C，退出时也执行清理
trap 'echo "[INFO] 用户手动终止，正在清理..."; nuclear_cleanup; exit 0' SIGINT

while true; do
    echo "=========================================="
    echo ">>> 准备启动新的导航会话..."
    
    # 每次启动前，先执行核弹以保平安
    nuclear_cleanup
    
    # 稍微等待进程完全释放
    sleep 1

    echo ">>> 启动 Launch 文件..."
    # ---------------------------------------------------------
    # 启动命令
    # 2>&1 | grep ... 用于过滤掉不想看的刷屏日志
    # ---------------------------------------------------------
    ros2 launch bringup monitored_start.launch.py 2>&1 | \
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
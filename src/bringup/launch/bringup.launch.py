from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
import os

def restart_nav_and_wait(cmds, target_action):
    """当导航节点退出时，重启导航 + 等待节点
       注意：不要复用已加入 LaunchDescription 的 ExecuteProcess 对象，重启时创建新实例
    """
    restart_actions = [ExecuteProcess(cmd=c, output="screen") for c in cmds]
    return RegisterEventHandler(
        OnProcessExit(
            target_action=target_action,  # 仅当指定目标退出时触发
            on_exit=restart_actions
        )
    )

def generate_launch_description():
    pkg_share = get_package_share_directory('bringup')

    # 原始命令列表（用于初次启动和重启时创建新的 ExecuteProcess）
    sensors_cmd = ["ros2", "launch", "bringup", "sensors.launch.py"]
    wait_scan_cmd = ["python3", os.path.join(pkg_share, "launch", "wait_for_topic_data.py"), "/scan", "6"]
    nav_cmd = ["ros2", "launch", "bringup", "navigation.launch.py"]
    wait_costmap_cmd = ["python3", os.path.join(pkg_share, "launch", "wait_for_topic_data.py"), "/local_costmap/costmap", "10"]
    wait_localmap_cmd = ["python3", os.path.join(pkg_share, "launch", "wait_for_topic_data.py"), "/local_map", "10"]
    serial_cmd = ["ros2", "launch", "bringup", "serial.launch.py"]
    bt_cmd = ["ros2", "launch", "bringup", "fake_bt.launch.py"]

    # 启动传感器节点
    sensors = ExecuteProcess(cmd=sensors_cmd, output="screen")

    # 等待 /scan
    wait_scan = ExecuteProcess(cmd=wait_scan_cmd, output="screen")

    # 启动导航节点
    nav = ExecuteProcess(cmd=nav_cmd, output="screen")

    # 等待地图话题
    wait_costmap = ExecuteProcess(cmd=wait_costmap_cmd, output="screen")
    wait_localmap = ExecuteProcess(cmd=wait_localmap_cmd, output="screen")

    wait_nodes_cmds = [wait_scan_cmd, wait_costmap_cmd, wait_localmap_cmd]

    # 后续节点
    serial = ExecuteProcess(cmd=serial_cmd, output="screen")
    bt = ExecuteProcess(cmd=bt_cmd, output="screen")

    # 注册重启事件：仅在 nav 退出时重启 nav + 等待节点（避免全局捕获导致重复执行）
    restart_cmds = [nav_cmd] + wait_nodes_cmds
    restart_event = restart_nav_and_wait(restart_cmds, nav)

    return LaunchDescription([
        sensors,
        wait_scan,
        nav,
        wait_costmap,
        wait_localmap,
        serial,
        bt,
        restart_event
    ])
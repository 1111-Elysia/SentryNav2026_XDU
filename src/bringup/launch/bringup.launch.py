from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
import os

def restart_nav_and_wait(nav, wait_nodes):
    """当导航节点或任何等待节点退出（超时）时，重启导航 + 所有等待节点"""
    actions = [nav] + wait_nodes
    return RegisterEventHandler(
        OnProcessExit(
            target_action=None,  # 捕获 nav 或任何等待节点退出
            on_exit=actions
        )
    )

def generate_launch_description():
    pkg_share = get_package_share_directory('bringup')

    # =======================
    # 启动传感器节点
    # =======================
    sensors = ExecuteProcess(
        cmd=["ros2", "launch", "bringup", "sensors.launch.py"],
        output="screen"
    )

    # =======================
    # 等待 /scan
    # =======================
    wait_scan = ExecuteProcess(
        cmd=["python3", os.path.join(pkg_share, "launch", "wait_for_topic_data.py"),
             "/scan", "6"],  # 超时时间可调整
        output="screen"
    )

    # =======================
    #  启动导航节点
    # =======================
    nav = ExecuteProcess(
        cmd=["ros2", "launch", "bringup", "navigation.launch.py"],
        output="screen"
    )

    # =======================
    # 等待地图话题
    # =======================
    wait_costmap = ExecuteProcess(
        cmd=["python3", os.path.join(pkg_share, "launch", "wait_for_topic_data.py"),
             "/local_costmap/costmap", "10"],
        output="screen"
    )

    wait_localmap = ExecuteProcess(
        cmd=["python3", os.path.join(pkg_share, "launch", "wait_for_topic_data.py"),
             "/local_map", "10"],
        output="screen"
    )

    wait_nodes = [wait_scan, wait_costmap, wait_localmap]

    # =======================
    # 后续节点
    # =======================
    serial = ExecuteProcess(
        cmd=["ros2", "launch", "bringup", "serial.launch.py"],
        output="screen"
    )

    bt = ExecuteProcess(
        cmd=["ros2", "launch", "bringup", "fake_bt.launch.py"],
        output="screen"
    )

    # =======================
    # 注册重启事件
    # 当导航节点或任意等待节点超时退出时，重启导航 + 所有等待节点
    # =======================
    restart_event = restart_nav_and_wait(nav, wait_nodes)

    # =======================
    # LaunchDescription 返回
    # =======================
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

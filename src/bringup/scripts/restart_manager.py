#!/usr/bin/env python3
"""
Restart Manager for Sentry Navigation
- Monitors costmaps and scan topics
- Handles system restart on failure
- Only kills processes in the same terminal (PGID)
- Closes UI window
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import psutil
import os
import signal
import subprocess
import threading
import time

class RestartManager(Node):
    def __init__(self):
        super().__init__('restart_manager')

        # =======================
        # 参数
        # =======================
        self.declare_parameter('restart_delay', 2.0)
        self.declare_parameter('max_restarts_per_hour', 10)
        self.restart_delay = self.get_parameter('restart_delay').value
        self.max_restarts = self.get_parameter('max_restarts_per_hour').value

        # 状态
        self.restarting = False
        self.restart_timestamps = []

        # 当前终端的 PGID
        self.pgid = os.getpgid(os.getpid())
        self.get_logger().info(f"RestartManager 启动，PGID={self.pgid}")

        # 订阅重启请求
        self.restart_sub = self.create_subscription(
            Bool,
            '/system/restart_request',
            self.handle_restart_request,
            10
        )

        self.get_logger().info("重启管理器启动成功")

    # =======================
    # 杀死 ROS 节点
    # =======================
    def cleanup_ros_nodes(self):
        """清理 launch 树下 ROS 节点"""
        self.get_logger().info("清理 ROS 节点...")
        lifecycle_nodes = [
            '/amcl','/bt_navigator','/controller_server',
            '/planner_server','/recoveries_server','/waypoint_follower'
        ]
        for node in lifecycle_nodes:
            try:
                subprocess.run(['ros2','lifecycle','set',node,'shutdown'],
                               timeout=2.0, capture_output=True)
            except:
                pass

        # 使用 psutil 遍历 PGID 下进程
        for proc in psutil.process_iter(['pid','name','cmdline']):
            try:
                if proc.pid == os.getpid():
                    continue
                proc_pgid = os.getpgid(proc.pid)
                if proc_pgid != self.pgid:
                    continue
                cmdline = ' '.join(proc.cmdline()) if proc.cmdline() else ''
                if any(k in cmdline.lower() for k in ['ros2','python','rviz','gazebo']):
                    self.get_logger().info(f"终止 ROS 进程 {proc.pid} {proc.name()}")
                    try:
                        proc.terminate()
                        proc.wait(timeout=1.0)
                    except:
                        try:
                            proc.kill()
                        except:
                            pass
            except (psutil.NoSuchProcess, psutil.AccessDenied, AttributeError):
                pass

    # =======================
    # 清理所有 ROS 相关进程（PGID）
    # =======================
    def kill_all_ros_processes(self):
        self.kill_ui_windows()
        self.cleanup_ros_nodes()

    # =======================
    # 重启整个系统
    # =======================
    def restart_entire_system(self):
        self.get_logger().error("触发系统重启，退出 launch 树...")
        time.sleep(0.5)
        os._exit(1)  # launch 会检测到非0退出自动重启

    # =======================
    # 重启请求回调
    # =======================
    def handle_restart_request(self, msg):
        if msg.data and not self.restarting:
            self.restarting = True
            self.get_logger().warn("收到重启请求，开始重启流程...")
            t = threading.Thread(target=self._perform_restart)
            t.daemon = True
            t.start()

    # =======================
    # 执行重启
    # =======================
    def _perform_restart(self):
        try:
            # 1. 关闭 UI 窗口
            # self.kill_ui_windows()
            # 2. 清理 ROS 节点
            self.cleanup_ros_nodes()
            time.sleep(1.0)
            # 3. 清理 ROS 进程
            self.kill_all_ros_processes()
            time.sleep(self.restart_delay)
            # 4. 重启系统
            self.restart_entire_system()
        except Exception as e:
            self.get_logger().error(f"重启过程中出错: {e}")
            self.restarting = False


def main(args=None):
    rclpy.init(args=args)
    manager = RestartManager()
    try:
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print("检测到 Ctrl+C，退出重启管理器")
    finally:
        manager.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

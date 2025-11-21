#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from sentry_msgs.msg import MatchStage
import subprocess
import signal
import os
import time

class FakeBtManager(Node):
    def __init__(self):
        super().__init__('fake_bt_manager')
        self.proc_point = None
        self.proc_vw = None
        self.sub = self.create_subscription(MatchStage, 'match_stage', self.cb, 10)
        self.get_logger().info('青春版行为树启动，等待比赛开始...')

    def cb(self, msg: MatchStage):
        try:
            stage = int(msg.match_stage)
        except Exception:
            stage = 0
        if stage == 4:
            self.start_nodes()
        else:
            self.stop_nodes()

    def start_nodes(self):
        if not self._is_running(self.proc_point):
            self.get_logger().info('开始发布路径点')
            points_yaml = os.path.join(
                get_package_share_directory('fake_bt'),
                'config',
                'points.yaml'
            )
            self.proc_point = subprocess.Popen([
                'ros2', 'run', 'fake_bt', 'pub_point',
                '--ros-args', '--params-file', points_yaml
            ])
        if not self._is_running(self.proc_vw):
            self.get_logger().info('小陀螺已上线')
            self.proc_vw = subprocess.Popen(['ros2', 'run', 'fake_bt', 'pub_vw'])

    def stop_nodes(self):
        if self._is_running(self.proc_point):
            self.get_logger().info('停止发布路径点')
            self._terminate(self.proc_point)
            self.proc_point = None
        if self._is_running(self.proc_vw):
            self.get_logger().info('停止小陀螺')
            self._terminate(self.proc_vw)
            self.proc_vw = None

    def _is_running(self, proc):
        return proc is not None and proc.poll() is None

    def _terminate(self, proc):
        try:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = FakeBtManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('关闭青春版行为树管理器，停止子进程...')
        node.stop_nodes()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

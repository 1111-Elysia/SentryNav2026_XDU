#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from sentry_msgs.msg import MatchStage
import subprocess
import os
import time

class FakeBtManager(Node):
    def __init__(self):
        super().__init__('fake_bt_manager')

        self.proc_point = None
        self.proc_vw = None
        self.nodes_started = False   # 🔹 防止重复启动

        self.sub = self.create_subscription(MatchStage, 'match_stage', self.cb, 10)
        self.get_logger().info('青春版行为树启动，等待比赛开始...')

        # 定时器检查子进程状态
        self.create_timer(1.0, self.check_subprocesses)

    # match_stage 回调
    def cb(self, msg: MatchStage):
        stage = int(msg.match_stage) if msg.match_stage else 0
        if stage == 4:
            self.start_nodes()
        else:
            self.stop_nodes()

    # 启动 pub_point 和 pub_vw
    def start_nodes(self):
        if self.nodes_started:
            return

        # 启动 pub_point
        if not self._is_running(self.proc_point):
            points_yaml = os.path.join(get_package_share_directory('fake_bt'), 'config', 'points.yaml')
            try:
                self.proc_point = subprocess.Popen(
                    ['ros2', 'run', 'fake_bt', 'pub_point', '--ros-args', '--params-file', points_yaml],
                    env=os.environ,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
                self.get_logger().info(f'开始发布路径点, pid={self.proc_point.pid}')
            except Exception as e:
                self.get_logger().error(f'启动 pub_point 失败: {e}')

        # 启动 pub_vw
        if not self._is_running(self.proc_vw):
            try:
                self.proc_vw = subprocess.Popen(
                    ['ros2', 'run', 'fake_bt', 'pub_vw'],
                    env=os.environ,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
                self.get_logger().info(f'小陀螺已上线, pid={self.proc_vw.pid}')
            except Exception as e:
                self.get_logger().error(f'启动 pub_vw 失败: {e}')

        self.nodes_started = True

    # 停止子进程
    def stop_nodes(self):
        if self._is_running(self.proc_point):
            self.get_logger().info('停止发布路径点')
            self._terminate(self.proc_point)
            self.proc_point = None

        if self._is_running(self.proc_vw):
            self.get_logger().info('停止小陀螺')
            self._terminate(self.proc_vw)
            self.proc_vw = None

        self.nodes_started = False

    # 检查子进程状态，异常退出自动重启
    def check_subprocesses(self):
        if not self.nodes_started:
            return

        if self.proc_point and self.proc_point.poll() is not None:
            self.get_logger().warn('pub_point 异常退出，尝试重启...')
            self.proc_point = None
            self.nodes_started = False
            self.start_nodes()

        if self.proc_vw and self.proc_vw.poll() is not None:
            self.get_logger().warn('pub_vw 异常退出，尝试重启...')
            self.proc_vw = None
            self.nodes_started = False
            self.start_nodes()

    # 判断子进程是否正在运行
    def _is_running(self, proc):
        return proc is not None and proc.poll() is None

    # 安全终止子进程
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

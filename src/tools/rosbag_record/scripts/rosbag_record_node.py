#!/usr/bin/env python3

import os
import signal
import subprocess
import time
from typing import List, Optional

import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rosidl_runtime_py.utilities import get_message


class RosbagRecordNode(Node):
    def __init__(self) -> None:
        super().__init__('rosbag_record')

        # NOTE: An empty Python list defaults to BYTE_ARRAY in rclpy.
        # Declare type explicitly so YAML string arrays load correctly.
        self.declare_parameter('topics', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('output_dir', '/tmp/rosbags')
        self.declare_parameter('bag_prefix', 'sentry')
        self.declare_parameter('start_game_progress', 3)
        self.declare_parameter('stop_game_progress', 5)
        self.declare_parameter('max_cache', 100)
        self.declare_parameter('stop_timeout_sec', 5.0)

        self._record_process: Optional[subprocess.Popen] = None
        self._record_output: Optional[str] = None

        msg_type = get_message('rm_referee_msgs/msg/GameStatus')
        self.create_subscription(
            msg_type,
            '/rm_referee/game_status',
            self._on_game_status,
            QoSProfile(depth=10),
        )

        self.get_logger().info('rosbag_record 节点已启动，等待 /rm_referee/game_status...')

    def _get_topics(self) -> List[str]:
        topics_param = self.get_parameter('topics').value
        if topics_param is None:
            return []
        if isinstance(topics_param, list):
            return [str(t).strip() for t in topics_param if str(t).strip()]
        return [str(topics_param).strip()] if str(topics_param).strip() else []

    def _is_process_running(self) -> bool:
        return self._record_process is not None and self._record_process.poll() is None

    def _start_recording(self) -> None:
        if self._is_process_running():
            return

        topics = self._get_topics()
        if not topics:
            self.get_logger().warn('参数 topics 为空：不会启动录包')
            return

        output_dir = str(self.get_parameter('output_dir').value)
        bag_prefix = str(self.get_parameter('bag_prefix').value)
        max_cache = int(self.get_parameter('max_cache').value)

        output_dir = os.path.expanduser(output_dir)
        if not os.path.isabs(output_dir):
            self.get_logger().error(
                f"参数 output_dir 必须是绝对路径，当前: '{output_dir}'（示例: /data/bags 或 /tmp/rosbags）"
            )
            return
        os.makedirs(output_dir, exist_ok=True)

        timestamp = time.strftime('%Y%m%d_%H%M%S')
        bag_name = f'{bag_prefix}_{timestamp}'
        output_path = os.path.join(output_dir, bag_name)

        cmd = ['ros2', 'bag', 'record', *topics, '--output', output_path, '--max-cache', str(max_cache)]

        try:
            self._record_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,
            )
        except FileNotFoundError:
            self._record_process = None
            self.get_logger().error('未找到 ros2 命令：请确认已 source ROS2 环境')
            return
        except Exception as exc:  # noqa: BLE001
            self._record_process = None
            self.get_logger().error(f'启动 rosbag 录制失败: {exc}')
            return

        self._record_output = output_path
        self.get_logger().info(f'开始录包: {output_path}')
        self.get_logger().info(f'录制话题: {topics}')

    def _stop_recording(self) -> None:
        if not self._is_process_running():
            self._record_process = None
            self._record_output = None
            return

        stop_timeout = float(self.get_parameter('stop_timeout_sec').value)

        proc = self._record_process
        assert proc is not None

        try:
            os.killpg(proc.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'发送 SIGINT 失败: {exc}')

        try:
            proc.wait(timeout=stop_timeout)
        except subprocess.TimeoutExpired:
            self.get_logger().warn('rosbag 未在超时内退出，尝试 SIGTERM...')
            try:
                os.killpg(proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'发送 SIGTERM 失败: {exc}')

            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.get_logger().warn('rosbag 仍未退出，尝试 SIGKILL...')
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(f'发送 SIGKILL 失败: {exc}')

        output = self._record_output
        self._record_process = None
        self._record_output = None
        if output:
            self.get_logger().info(f'停止录包: {output}')
        else:
            self.get_logger().info('停止录包')

    def _on_game_status(self, msg) -> None:
        start_progress = int(self.get_parameter('start_game_progress').value)
        stop_progress = int(self.get_parameter('stop_game_progress').value)

        progress = int(getattr(msg, 'game_progress'))
        if progress == start_progress:
            self._start_recording()
        elif progress == stop_progress:
            self._stop_recording()

    def destroy_node(self) -> bool:
        try:
            self._stop_recording()
        finally:
            return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = RosbagRecordNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

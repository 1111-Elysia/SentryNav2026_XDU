#!/usr/bin/env python3
"""
重启管理器 - 处理节点关闭和系统重启
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import subprocess
import os
import signal
import time
import psutil
import threading
import shutil

class RestartManager(Node):
    def __init__(self):
        super().__init__('restart_manager')
        
        # 参数
        self.declare_parameter('restart_delay', 2.0)
        self.declare_parameter('max_restarts_per_hour', 10)
        
        # 获取参数
        self.restart_delay = self.get_parameter('restart_delay').value
        self.max_restarts = self.get_parameter('max_restarts_per_hour').value
        
        # 记录当前终端TTY和会话SID
        self.current_tty, self.current_sid = self._get_current_tty_and_sid()
        self.get_logger().info(f'重启管理器启动（限制到 TTY={self.current_tty}, SID={self.current_sid}）')

        # 重启历史
        self.restart_timestamps = []
        self.restarting = False
        
        # 订阅重启请求
        self.restart_sub = self.create_subscription(
            Bool,
            '/system/restart_request',
            self.handle_restart_request,
            10
        )
        
        self.get_logger().info('重启管理器启动')
    
    def _get_current_tty_and_sid(self):
        """获取当前进程的TTY和会话SID（如果本进程没有TTY则尝试父进程）"""
        try:
            p = psutil.Process(os.getpid())
            tty = p.terminal()
            if not tty:
                for parent in p.parents():
                    tty = parent.terminal()
                    if tty:
                        break
            sid = os.getsid(os.getpid())
            return tty, sid
        except Exception:
            return None, os.getsid(os.getpid())
        
    def _belongs_to_current_session(self, proc: psutil.Process) -> bool:
        """只处理同一TTY或同一会话SID的进程"""
        try:
            if self.current_tty:
                if proc.terminal() == self.current_tty:
                    return True
            return os.getsid(proc.pid) == self.current_sid
        except Exception:
            return False

    def _is_terminal_or_shell(self, proc: psutil.Process) -> bool:
        """避免杀掉终端/外壳/VS Code等宿主"""
        try:
            name = (proc.name() or '').lower()
            cmd0 = ''
            try:
                cmd = proc.cmdline()
                cmd0 = (cmd[0] if cmd else '').lower()
            except Exception:
                pass
            safe = {
                'bash','zsh','fish','sh','tmux','screen',
                'gnome-terminal','konsole','xterm','kitty',
                'alacritty','wezterm','code','code-oss','python','python3'
            }
            return name in safe or cmd0 in safe
        except Exception:
            return False

    def kill_ui_windows(self):
        """
        杀掉当前TTY/会话中的UI窗口
        1. PID>0 用 psutil terminate/kill
        2. PID=0 或特殊窗口标题，用 wmctrl 或 xdotool 尝试关闭
        """
        try:
            self.get_logger().info("开始关闭UI窗口...")
            # Step 1: 使用 wmctrl 列出窗口
            wmctrl_available = shutil.which('wmctrl') is not None
            xdotool_available = shutil.which('xdotool') is not None

            if wmctrl_available:
                out = subprocess.run(['wmctrl', '-lp'], capture_output=True, text=True, timeout=2.0)
                for line in out.stdout.splitlines():
                    parts = line.split()
                    if len(parts) < 5:
                        continue
                    win_id, desktop, pid_str, host = parts[:4]
                    title = ' '.join(parts[4:])
                    
                    # PID>0，尝试杀 ROS/UI 进程
                    if pid_str.isdigit() and int(pid_str) > 0:
                        pid = int(pid_str)
                        try:
                            proc = psutil.Process(pid)
                            if self._is_terminal_or_shell(proc):
                                continue
                            self.get_logger().info(f"终止窗口进程 PID={pid} 名称={proc.name()}")
                            proc.terminate()
                            proc.wait(timeout=1.0)
                        except Exception:
                            try:
                                psutil.Process(pid).kill()
                            except Exception:
                                pass
                    else:
                        # PID=0 或特殊标题
                        if 'UI' in title or 'N/A UI' in title:
                            self.get_logger().info(f"尝试关闭无PID窗口 {win_id} 标题={title}")
                            try:
                                if wmctrl_available:
                                    subprocess.run(['wmctrl', '-i', '-c', win_id], timeout=1.0)
                                elif xdotool_available:
                                    subprocess.run(['xdotool', 'windowclose', win_id], timeout=1.0)
                            except Exception as e:
                                self.get_logger().warn(f"关闭窗口失败 {win_id}: {e}")

            self.get_logger().info("UI窗口关闭完成。")
        except Exception as e:
            self.get_logger().error(f"关闭UI窗口失败: {e}")

    def cleanup_ros_nodes(self):
        """清理所有ROS节点"""
        self.get_logger().info('开始清理ROS节点...')
        try:
            # 发送 lifecycle shutdown
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
            
            # kill ROS nodes
            try:
                nodes_result = subprocess.run(['ros2','node','list'], capture_output=True, text=True, timeout=5.0)
                if nodes_result.stdout:
                    nodes = nodes_result.stdout.strip().split('\n')
                    self.get_logger().info(f'找到节点: {nodes}')
                    for node in nodes:
                        if node:
                            try:
                                info_result = subprocess.run(['ros2','node','info',node],
                                                             capture_output=True, text=True, timeout=3.0)
                                for line in info_result.stdout.split('\n'):
                                    if 'PID:' in line:
                                        pid = line.split(':')[-1].strip()
                                        if pid.isdigit():
                                            os.kill(int(pid), signal.SIGINT)
                                            self.get_logger().info(f'发送SIGINT到节点 {node} (PID: {pid})')
                            except Exception as e:
                                self.get_logger().warn(f'无法停止节点 {node}: {e}')
            except Exception as e:
                self.get_logger().error(f'获取节点列表失败: {e}')
        except Exception as e:
            self.get_logger().error(f'清理节点时出错: {e}')

    def kill_all_ros_processes(self):
        """杀死当前TTY/会话中的ROS相关进程"""
        self.get_logger().info('清理ROS进程（仅限当前TTY/会话）...')
        try:
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                try:
                    if proc.pid == os.getpid():
                        continue
                    if not self._belongs_to_current_session(proc):
                        continue
                    cmdline = ' '.join(proc.cmdline()) if proc.cmdline() else ''
                    lower = cmdline.lower()
                    is_ros_process = any(keyword in lower for keyword in [
                        'ros2','roscore','rviz','rviz2','gazebo','rqt',
                        'nav2','amcl','bt_navigator','controller_server',
                        'planner_server','recoveries_server','waypoint_follower',
                        'livox','fast_lio','lightning','navigation'
                    ])
                    if is_ros_process:
                        self.get_logger().info(f'终止进程: {proc.pid} - {proc.name()} ({proc.terminal()})')
                        try:
                            proc.terminate()
                            proc.wait(timeout=1.0)
                        except Exception:
                            try:
                                proc.kill()
                            except Exception:
                                pass
                except (psutil.NoSuchProcess, psutil.AccessDenied, AttributeError):
                    pass

            # 可选回退命令
            if self.current_tty:
                cleanup_commands = [
                    ['pkill', '-t', self.current_tty, '-f', 'ros2'],
                    ['pkill', '-t', self.current_tty, '-f', 'rviz2'],
                    ['pkill', '-t', self.current_tty, '-f', 'gazebo'],
                    ['pkill', '-t', self.current_tty, '-f', 'livox'],
                    ['pkill', '-t', self.current_tty, '-f', 'fast_lio'],
                    ['pkill', '-t', self.current_tty, '-f', 'lightning'],
                    ['pkill', '-t', self.current_tty, '-f', 'navigation'],
                ]
                for cmd in cleanup_commands:
                    try:
                        subprocess.run(cmd, timeout=1.0, capture_output=True)
                    except Exception:
                        pass
            time.sleep(1.0)
        except Exception as e:
            self.get_logger().error(f'清理进程时出错: {e}')

    def restart_entire_system(self):
        self.get_logger().error("Requesting full system restart. Exiting launch tree.")
        time.sleep(0.5)
        os._exit(1)

    def handle_restart_request(self, msg):
        """处理重启请求"""
        if msg.data and not self.restarting:
            self.restarting = True
            self.get_logger().warn('收到重启请求，开始重启流程')
            restart_thread = threading.Thread(target=self._perform_restart)
            restart_thread.daemon = True
            restart_thread.start()
    
    def _perform_restart(self):
        """执行重启流程"""
        try:
            self.kill_ui_windows()
            self.cleanup_ros_nodes()
            time.sleep(1.0)
            self.kill_all_ros_processes()
            time.sleep(self.restart_delay)
            self.restart_entire_system()
        except Exception as e:
            self.get_logger().error(f'重启过程中出错: {e}')
            self.restarting = False

def main(args=None):
    rclpy.init(args=args)
    manager = RestartManager()
    try:
        rclpy.spin(manager)
    except KeyboardInterrupt:
        pass
    finally:
        manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

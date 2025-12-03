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
from datetime import datetime
import sys

class RestartManager(Node):
    def __init__(self):
        super().__init__('restart_manager')
        
        # 参数
        self.declare_parameter('restart_delay', 2.0)
        self.declare_parameter('max_restarts_per_hour', 10)
        
        # 获取参数
        self.restart_delay = self.get_parameter('restart_delay').value
        self.max_restarts = self.get_parameter('max_restarts_per_hour').value
        
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
    
    def cleanup_ros_nodes(self):
        """清理所有ROS节点 - 使用正确的ROS 2命令"""
        self.get_logger().info('开始清理ROS节点...')
        
        try:
            # 方法1: 使用ros2 lifecycle命令
            lifecycle_nodes = [
                '/amcl',
                '/bt_navigator',
                '/controller_server',
                '/planner_server',
                '/recoveries_server',
                '/waypoint_follower'
            ]
            
            for node in lifecycle_nodes:
                try:
                    subprocess.run(['ros2', 'lifecycle', 'set', node, 'shutdown'], 
                                 timeout=2.0, capture_output=True)
                except:
                    pass
            
            # 方法2: 使用kill命令发送信号
            try:
                # 获取所有节点进程
                nodes_result = subprocess.run(['ros2', 'node', 'list'], 
                                            capture_output=True, text=True, timeout=5.0)
                
                if nodes_result.stdout:
                    nodes = nodes_result.stdout.strip().split('\n')
                    self.get_logger().info(f'找到节点: {nodes}')
                    
                    # 发送SIGINT信号
                    for node in nodes:
                        if node:
                            try:
                                # 获取节点PID
                                info_result = subprocess.run(['ros2', 'node', 'info', node],
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
        """杀死所有ROS相关进程"""
        self.get_logger().info('清理ROS进程...')
        
        try:
            # 查找并杀死所有ros2、rviz2、gazebo等进程
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                try:
                    if proc.pid == os.getpid():  # 跳过自己
                        continue
                        
                    cmdline = ' '.join(proc.cmdline()) if proc.cmdline() else ''
                    
                    # 判断是否是ROS相关进程
                    is_ros_process = any(keyword in cmdline.lower() for keyword in [
                        'ros2', 'roscore', 'rviz', 'gazebo', 'python3',
                        'livox', 'fast_lio', 'lightning', 'navigation'
                    ])
                    
                    if is_ros_process:
                        self.get_logger().info(f'终止进程: {proc.pid} - {proc.name()}')
                        try:
                            proc.terminate()  # 先尝试正常终止
                            proc.wait(timeout=1.0)
                        except:
                            try:
                                proc.kill()  # 强制终止
                            except:
                                pass
                except (psutil.NoSuchProcess, psutil.AccessDenied, AttributeError):
                    pass
            
            # 额外的清理命令
            cleanup_commands = [
                ['pkill', '-f', 'ros2'],
                ['pkill', '-f', 'rviz2'],
                ['pkill', '-f', 'gazebo'],
                ['pkill', '-f', 'python.*livox'],
                ['pkill', '-f', 'python.*navigation'],
                ['pkill', '-f', 'fast_lio'],
                ['pkill', '-f', 'lightning'],
            ]
            
            for cmd in cleanup_commands:
                try:
                    subprocess.run(cmd, timeout=1.0, capture_output=True)
                except:
                    pass
            
            time.sleep(1.0)  # 等待清理完成
            
        except Exception as e:
            self.get_logger().error(f'清理进程时出错: {e}')
    
    def restart_entire_system(self):
        """重启整个系统（外部脚本方式）"""
        self.get_logger().info('执行外部重启脚本...')
        
        try:
            # 创建一个重启脚本并执行
            restart_script = '''
#!/bin/bash
# 等待一小段时间让当前进程清理
sleep 2

# 清理所有ROS进程
pkill -f ros2
pkill -f rviz2
pkill -f python3.*ros
sleep 1

# 重新启动系统
cd ~/SentryNav2026_XDU
source install/setup.bash
ros2 launch bringup monitored_start.launch.py &
exit 0
'''
            
            # 将脚本写入临时文件
            script_path = '/tmp/restart_nav_system.sh'
            with open(script_path, 'w') as f:
                f.write(restart_script)
            
            os.chmod(script_path, 0o755)
            
            # 在新的进程中执行重启脚本
            subprocess.Popen(['bash', script_path],
                           stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL,
                           preexec_fn=os.setsid)
            
            # 当前进程可以退出了
            self.get_logger().info('重启脚本已启动，管理器退出')
            
            # 延迟退出，确保消息发送完成
            time.sleep(0.5)
            
            # 自我终止
            os._exit(0)
            
        except Exception as e:
            self.get_logger().error(f'重启失败: {e}')
            self.restarting = False
    
    def handle_restart_request(self, msg):
        """处理重启请求"""
        if msg.data and not self.restarting:
            self.restarting = True
            self.get_logger().warn('收到重启请求，开始重启流程')
            
            # 在新线程中执行重启
            restart_thread = threading.Thread(target=self._perform_restart)
            restart_thread.daemon = True
            restart_thread.start()
    
    def _perform_restart(self):
        """执行重启流程"""
        try:
            # 1. 清理ROS节点
            self.cleanup_ros_nodes()
            
            # 2. 等待
            time.sleep(1.0)
            
            # 3. 清理所有进程
            self.kill_all_ros_processes()
            
            # 4. 等待清理完成
            time.sleep(self.restart_delay)
            
            # 5. 重启整个系统
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
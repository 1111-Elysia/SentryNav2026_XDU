#!/usr/bin/env python3
"""
监控全局和局部costmap话题的活跃度
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
import time
from datetime import datetime
import threading

class CostmapMonitor(Node):
    def __init__(self):
        super().__init__('costmap_monitor')
        
        # 参数
        self.declare_parameter('global_costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('local_costmap_topic', '/local_costmap/costmap')
        self.declare_parameter('timeout_seconds', 5.0)
        self.declare_parameter('check_interval', 1.0)
        self.declare_parameter('max_consecutive_timeouts', 3)
        
        # 获取参数
        self.global_topic = self.get_parameter('global_costmap_topic').value
        self.local_topic = self.get_parameter('local_costmap_topic').value
        self.timeout_seconds = self.get_parameter('timeout_seconds').value
        self.check_interval = self.get_parameter('check_interval').value
        self.max_timeouts = self.get_parameter('max_consecutive_timeouts').value
        
        # 状态变量
        self.last_global_update = time.time()
        self.last_local_update = time.time()
        self.global_active = False
        self.local_active = False
        self.consecutive_timeouts = 0
        self.monitoring_active = True
        
        # 订阅者
        self.global_sub = self.create_subscription(
            OccupancyGrid,
            self.global_topic,
            self.global_callback,
            10
        )
        
        self.local_sub = self.create_subscription(
            OccupancyGrid,
            self.local_topic,
            self.local_callback,
            10
        )
        
        # 发布者 - 发布系统状态
        self.status_pub = self.create_publisher(Bool, '/system/health_status', 10)
        
        # 定时器检查活跃度
        self.check_timer = self.create_timer(self.check_interval, self.check_costmaps)
        
        # 日志
        self.get_logger().info(f'监控器启动，监控话题: {self.global_topic}, {self.local_topic}')
        self.get_logger().info(f'超时时间: {self.timeout_seconds}秒，检查间隔: {self.check_interval}秒')
    
    def global_callback(self, msg):
        """全局costmap回调"""
        self.last_global_update = time.time()
        if not self.global_active:
            self.global_active = True
            self.get_logger().info('全局costmap开始更新')
    
    def local_callback(self, msg):
        """局部costmap回调"""
        self.last_local_update = time.time()
        if not self.local_active:
            self.local_active = True
            self.get_logger().info('局部costmap开始更新')
    
    def check_costmaps(self):
        """检查costmap更新状态"""
        current_time = time.time()
        
        # 检查全局costmap
        global_timeout = current_time - self.last_global_update > self.timeout_seconds
        local_timeout = current_time - self.last_local_update > self.timeout_seconds
        
        if global_timeout or local_timeout:
            self.consecutive_timeouts += 1
            timeout_msg = []
            if global_timeout:
                timeout_msg.append('全局costmap')
            if local_timeout:
                timeout_msg.append('局部costmap')
            
            self.get_logger().warn(
                f'警告: {", ".join(timeout_msg)}超过{self.timeout_seconds}秒未更新 '
                f'(连续超时: {self.consecutive_timeouts}/{self.max_timeouts})'
            )
            
            # 检查是否需要触发重启
            if self.consecutive_timeouts >= self.max_timeouts:
                self.trigger_restart()
        else:
            # 重置连续超时计数
            if self.consecutive_timeouts > 0:
                self.get_logger().info('costmap恢复更新，重置超时计数')
                self.consecutive_timeouts = 0
        
        # 发布系统健康状态
        status_msg = Bool()
        status_msg.data = self.consecutive_timeouts < self.max_timeouts
        self.status_pub.publish(status_msg)
    
    def trigger_restart(self):
        """触发系统重启"""
        self.get_logger().error('检测到costmap持续未更新，触发系统重启！')
        
        # 发布重启信号
        restart_pub = self.create_publisher(Bool, '/system/restart_request', 10)
        restart_msg = Bool()
        restart_msg.data = True
        restart_pub.publish(restart_msg)
        
        # 重置计数器
        self.consecutive_timeouts = 0
        
        # 等待重启管理器处理
        time.sleep(2.0)

def main(args=None):
    rclpy.init(args=args)
    monitor = CostmapMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
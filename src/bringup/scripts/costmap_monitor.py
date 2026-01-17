#!/usr/bin/env python3
"""
监控全局和局部costmap话题的活跃度，并在连续超时后触发系统重启
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
import time
from datetime import datetime
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class CostmapMonitor(Node):
    def __init__(self):
        super().__init__('costmap_monitor')

        # 参数
        self.declare_parameter('global_costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('local_costmap_topic', '/local_costmap/costmap')
        self.declare_parameter('timeout_seconds', 5.0)
        self.declare_parameter('check_interval', 1.0)
        self.declare_parameter('max_consecutive_timeouts', 3)
        self.declare_parameter('warn_log_interval', 2.0)

        # 获取参数
        self.global_topic = self.get_parameter('global_costmap_topic').value
        self.local_topic = self.get_parameter('local_costmap_topic').value
        self.timeout_seconds = self.get_parameter('timeout_seconds').value
        self.check_interval = self.get_parameter('check_interval').value
        self.max_timeouts = self.get_parameter('max_consecutive_timeouts').value
        self.warn_log_interval = self.get_parameter('warn_log_interval').value

        # 状态变量
        self.last_global_update = time.time()
        self.last_local_update = time.time()
        self.consecutive_timeouts = 0
        self.restart_requested = False
        self.last_warn_time = 0.0

        # QoS 设置，保证 late subscriber 收到重启消息
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 订阅 costmap
        self.global_sub = self.create_subscription(
            OccupancyGrid, self.global_topic, self.global_callback, qos)
        self.local_sub = self.create_subscription(
            OccupancyGrid, self.local_topic, self.local_callback, qos)

        # 发布系统健康状态和重启请求
        self.status_pub = self.create_publisher(Bool, '/system/health_status', qos)
        self.restart_pub = self.create_publisher(Bool, '/system/restart_request', qos)

        # 定时器
        self.check_timer = self.create_timer(self.check_interval, self.check_costmaps)

        self.get_logger().info(f"{datetime.now()} - CostmapMonitor 启动，监控话题: {self.global_topic}, {self.local_topic}")
        self.get_logger().info(f"{datetime.now()} - 超时时间: {self.timeout_seconds}s, 检查间隔: {self.check_interval}s")

    def global_callback(self, msg):
        self.last_global_update = time.time()

    def local_callback(self, msg):
        self.last_local_update = time.time()

    def check_costmaps(self):
        current_time = time.time()
        global_timeout = current_time - self.last_global_update > self.timeout_seconds
        local_timeout = current_time - self.last_local_update > self.timeout_seconds

        if global_timeout or local_timeout:
            self.consecutive_timeouts += 1

            # warn日志
            if current_time - self.last_warn_time >= self.warn_log_interval:
                timeout_msg = []
                if global_timeout:
                    timeout_msg.append('全局costmap')
                if local_timeout:
                    timeout_msg.append('局部costmap')
                self.get_logger().warn(
                    f"{datetime.now()} - 警告: {', '.join(timeout_msg)} 超过 {self.timeout_seconds}s 未更新 "
                    f"(连续超时 {self.consecutive_timeouts}/{self.max_timeouts})"
                )
                self.last_warn_time = current_time

            # 达到阈值触发重启
            if self.consecutive_timeouts >= self.max_timeouts and not self.restart_requested:
                self.trigger_restart()
        else:
            if self.consecutive_timeouts > 0:
                self.get_logger().info(f"{datetime.now()} - costmap恢复更新，重置超时计数")
            self.consecutive_timeouts = 0
            # 不复位 restart_requested，让 restart_manager 收到消息处理重启

        # 发布健康状态
        status_msg = Bool()
        status_msg.data = self.consecutive_timeouts < self.max_timeouts
        self.status_pub.publish(status_msg)

    def trigger_restart(self):
        self.get_logger().error(f"{datetime.now()} - costmap连续超时，触发系统重启！")
        msg = Bool()
        msg.data = True
        self.restart_pub.publish(msg)
        self.restart_requested = True  # 保持 True，直到 restart_manager 执行
        # 不清零 consecutive_timeouts，保证连续逻辑正确

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

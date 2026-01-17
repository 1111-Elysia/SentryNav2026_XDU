#!/usr/bin/env python3
"""
Scan Monitor - 监控激光扫描数据健康状态
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import time

class ScanMonitor(Node):
    def __init__(self):
        super().__init__('scan_monitor')

        # 参数
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('timeout_seconds', 5.0)
        self.declare_parameter('check_interval', 1.0)
        self.declare_parameter('max_consecutive_timeouts', 3)

        # 获取参数
        self.scan_topic = self.get_parameter('scan_topic').value
        self.timeout_seconds = self.get_parameter('timeout_seconds').value
        self.check_interval = self.get_parameter('check_interval').value
        self.max_timeouts = self.get_parameter('max_consecutive_timeouts').value

        # 状态变量
        self.last_update = None
        self.consecutive_timeouts = 0

        # 订阅者
        self.sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )

        # 发布系统健康状态（可选）
        self.status_pub = self.create_publisher(Bool, '/system/scan_health', 10)

        # 定时器检查
        self.create_timer(self.check_interval, self.check_scan)

        self.get_logger().info(f'ScanMonitor 启动，监控话题: {self.scan_topic}')

    def scan_callback(self, msg: LaserScan):
        """收到 scan 消息时触发"""
        self.last_update = time.time()

        # 检查 scan 数据是否全为 inf 或 0
        values = [v for v in msg.ranges if v is not None]
        if not values:
            self.get_logger().warn('Scan 数据为空，可能传感器异常')
            self.consecutive_timeouts += 1
            return

        all_inf = all([v == float('inf') for v in values])
        all_zero = all([v == 0.0 for v in values])

        if all_inf or all_zero:
            self.get_logger().warn('Scan 数据全为 inf 或 0')
            self.consecutive_timeouts += 1
        else:
            # 数据正常，重置计数
            if self.consecutive_timeouts > 0:
                self.get_logger().info('Scan 数据恢复正常')
                self.consecutive_timeouts = 0

    def check_scan(self):
        """定时检查 scan 是否超时"""
        now = time.time()
        if self.last_update is None or (now - self.last_update) > self.timeout_seconds:
            self.consecutive_timeouts += 1
            self.get_logger().warn(
                f'Scan 超时未更新 (超过 {self.timeout_seconds}s)，连续超时 {self.consecutive_timeouts}/{self.max_timeouts}'
            )
        else:
            # 已更新，连续超时计数保持
            pass

        # 发布健康状态
        status = Bool()
        status.data = self.consecutive_timeouts < self.max_timeouts
        self.status_pub.publish(status)

        # 超过连续阈值 → 触发重启
        if self.consecutive_timeouts >= self.max_timeouts:
            self.trigger_restart()

    def trigger_restart(self):
        """触发系统重启"""
        self.get_logger().error('Scan 持续异常，触发系统重启！')

        # 发布重启请求
        restart_pub = self.create_publisher(Bool, '/system/restart_request', 10)
        restart_msg = Bool()
        restart_msg.data = True
        restart_pub.publish(restart_msg)

        # 重置计数器
        self.consecutive_timeouts = 0

def main(args=None):
    rclpy.init(args=args)
    node = ScanMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

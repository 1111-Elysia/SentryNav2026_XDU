#!/usr/bin/env python3
"""
Map 监控器 - 当 /map 未发布或为空时触发重启
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
import time
import threading

class MapMonitor(Node):
    def __init__(self):
        super().__init__('map_monitor')

        # 参数
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('timeout_seconds', 10.0)  # 最长多久没发布触发重启
        self.declare_parameter('check_interval', 1.0)    # 检查间隔
        self.declare_parameter('max_empty_counts', 3)    # 连续空数据次数触发重启

        self.map_topic = self.get_parameter('map_topic').value
        self.timeout_seconds = self.get_parameter('timeout_seconds').value
        self.check_interval = self.get_parameter('check_interval').value
        self.max_empty_counts = self.get_parameter('max_empty_counts').value

        self.last_update_time = time.time()
        self.empty_count = 0

        # 订阅 /map
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            10
        )

        # 发布重启请求
        self.restart_pub = self.create_publisher(Bool, '/system/restart_request', 10)

        # 定时器检查
        self.timer = self.create_timer(self.check_interval, self.check_map)

        self.get_logger().info(f"MapMonitor 启动，监控话题: {self.map_topic}")

    def map_callback(self, msg: OccupancyGrid):
        """收到 map 数据"""
        self.last_update_time = time.time()

        # 检查是否为空
        if not msg.data or all(v == 0 for v in msg.data):
            self.empty_count += 1
            self.get_logger().warn(f"/map 数据为空，累计 {self.empty_count}/{self.max_empty_counts}")
        else:
            self.empty_count = 0  # 重置空计数

    def check_map(self):
        """定期检查 map 是否超时或为空"""
        current_time = time.time()
        elapsed = current_time - self.last_update_time

        if elapsed > self.timeout_seconds:
            self.get_logger().error(f"/map {elapsed:.1f}s 未更新，触发重启")
            self.trigger_restart()
            return

        if self.empty_count >= self.max_empty_counts:
            self.get_logger().error(f"/map 连续 {self.empty_count} 次为空，触发重启")
            self.trigger_restart()
            return

    def trigger_restart(self):
        """发布重启信号"""
        msg = Bool()
        msg.data = True
        self.restart_pub.publish(msg)
        self.get_logger().info("已发布 /system/restart_request")
        self.empty_count = 0
        self.last_update_time = time.time()  # 防止重复触发

def main(args=None):
    rclpy.init(args=args)
    monitor = MapMonitor()
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

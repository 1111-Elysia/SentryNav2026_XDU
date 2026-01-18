#!/usr/bin/env python3
"""
Scan Monitor - 监控激光扫描数据健康状态
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class ScanMonitor(Node):
    def __init__(self):
        super().__init__('scan_monitor')

        # 参数
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('timeout_seconds', 5.0)
        self.declare_parameter('check_interval', 1.0)
        self.declare_parameter('max_consecutive_timeouts', 3)
        # 可选：判定“全 inf/0”过严时，改用占比阈值（0~1），默认 98%
        self.declare_parameter('bad_ratio_threshold', 0.98)
        # 新增：启动宽限时间（在此时间内不判定超时）
        self.declare_parameter('startup_grace_seconds', 3.0)

        # 获取参数
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.timeout_seconds = self.get_parameter('timeout_seconds').get_parameter_value().double_value
        self.check_interval = self.get_parameter('check_interval').get_parameter_value().double_value
        self.max_timeouts = self.get_parameter('max_consecutive_timeouts').get_parameter_value().integer_value
        self.bad_ratio_threshold = float(self.get_parameter('bad_ratio_threshold').value)
        self.startup_grace_seconds = self.get_parameter('startup_grace_seconds').get_parameter_value().double_value

        # 状态变量
        self.last_update = None
        self.consecutive_timeouts = 0
        self.start_time = time.monotonic()
        self.seen_first_scan = False

        # 订阅者（使用 sensor data 常见的 Best Effort QoS）
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        self.sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            sensor_qos
        )

        # 发布系统健康状态 & 重启请求（复用发布者）
        self.status_pub = self.create_publisher(Bool, '/system/scan_health', 10)
        self.restart_pub = self.create_publisher(Bool, '/system/restart_request', 10)

        # 定时器检查
        self.create_timer(self.check_interval, self.check_scan)

        self.get_logger().info(f'ScanMonitor 启动，监控话题: {self.scan_topic}, QoS=BestEffort')

    def scan_callback(self, msg: LaserScan):
        """收到 scan 消息时触发"""
        # 使用单调时钟避免系统时钟跳变
        self.last_update = time.monotonic()
        self.seen_first_scan = True

        # 过滤 None/NaN
        values = [v for v in msg.ranges if v is not None and not math.isnan(v)]
        if not values:
            self.get_logger().warn('Scan 数据为空或全为 NaN，可能传感器异常')
            self.consecutive_timeouts += 1
            return

        # 计算 inf 与 0 的占比
        n = len(values)
        inf_ratio = sum(1 for v in values if math.isinf(v)) / n
        zero_ratio = sum(1 for v in values if v == 0.0) / n
        bad_ratio = max(inf_ratio, zero_ratio)

        if bad_ratio >= self.bad_ratio_threshold:
            self.get_logger().warn(
                f'Scan 异常占比过高 (>= {self.bad_ratio_threshold:.2f}) '
                f'[inf={inf_ratio:.2f}, zero={zero_ratio:.2f}]'
            )
            self.consecutive_timeouts += 1
        else:
            # 数据正常，重置计数
            if self.consecutive_timeouts > 0:
                self.get_logger().info('Scan 数据恢复正常')
            self.consecutive_timeouts = 0

    def check_scan(self):
        """定时检查 scan 是否超时"""
        now = time.monotonic()

        # 首帧未到达：超过启动宽限后开始计超时
        if not self.seen_first_scan:
            elapsed = now - self.start_time

            if elapsed <= self.startup_grace_seconds:
                status = Bool()
                status.data = True
                self.status_pub.publish(status)
                return

            # 超过宽限仍未收到首帧 → 计入超时
            self.consecutive_timeouts += 1
            try:
                pub_count = len(self.get_publishers_info_by_topic(self.scan_topic))
            except Exception:
                pub_count = -1

            if pub_count == 0:
                self.get_logger().warn(
                    f'{self.scan_topic} 没有发布者，已等待 {elapsed:.1f}s，连续超时 {self.consecutive_timeouts}/{self.max_timeouts}'
                )
            else:
                self.get_logger().warn(
                    f'首帧未到达，已等待 {elapsed:.1f}s，连续超时 {self.consecutive_timeouts}/{self.max_timeouts}'
                )

            status = Bool()
            status.data = self.consecutive_timeouts < self.max_timeouts
            self.status_pub.publish(status)

            if self.consecutive_timeouts >= self.max_timeouts:
                self.trigger_restart()
            return

        # 已收到首帧后的常规超时判断
        if self.last_update is None or (now - self.last_update) > self.timeout_seconds:
            self.consecutive_timeouts += 1
            self.get_logger().warn(
                f'Scan 超时未更新 (超过 {self.timeout_seconds}s)，'
                f'连续超时 {self.consecutive_timeouts}/{self.max_timeouts}'
            )

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
        restart_msg = Bool()
        restart_msg.data = True
        self.restart_pub.publish(restart_msg)
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
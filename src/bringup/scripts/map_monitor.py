#!/usr/bin/env python3
"""
Map 监控器 - 适用于 Nav2 静态地图
只在第一次接收到无效地图时触发重启
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class MapMonitor(Node):
    def __init__(self):
        super().__init__('map_monitor')

        # 参数
        self.declare_parameter('map_topic', '/map')
        self.map_topic = self.get_parameter('map_topic').value

        # 标记是否已经处理地图
        self.map_checked = False

        # 发布重启请求
        self.restart_pub = self.create_publisher(Bool, '/system/restart_request', 10)

        # QoS：Transient Local 保证晚启动也能收到地图
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        # 订阅 /map
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            qos
        )

        self.get_logger().info(f"MapMonitor 启动，监控话题: {self.map_topic}")

    def map_callback(self, msg: OccupancyGrid):
        if self.map_checked:
            return  # 只检查一次

        # 检查地图有效性
        if not msg.data or all(v <= -1 for v in msg.data):
            self.get_logger().error("收到空地图或全未知地图，触发重启")
            self.trigger_restart()
        else:
            self.get_logger().info("地图接收成功，无需重启")

        self.map_checked = True  # 只处理一次

    def trigger_restart(self):
        msg = Bool()
        msg.data = True
        self.restart_pub.publish(msg)
        self.get_logger().info("已发布 /system/restart_request")

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

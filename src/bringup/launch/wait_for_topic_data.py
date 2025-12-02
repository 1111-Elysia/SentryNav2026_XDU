import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import sys

class WaitForTopicData(Node):
    """
    监听指定话题，检测到话题存在就退出。
    超时退出，用于 launch 捕获重启。
    （不依赖 AnyMsg，避免在某些环境中导入失败）
    """
    def __init__(self, topic_name, timeout_sec):
        super().__init__(f"wait_for_{topic_name.replace('/', '_')}")
        self.topic_name = topic_name
        self.timeout_sec = timeout_sec
        self.data_received = False
        self.deadline = self.get_clock().now() + Duration(seconds=timeout_sec)

        # 周期性检查话题是否存在 / 有发布者
        self.timer = self.create_timer(0.1, self.check_timeout)

    def check_timeout(self):
        if self.data_received:
            return

        # 如果话题名出现在系统话题列表中，认为已开始发布
        topics = self.get_topic_names_and_types()
        if any(name == self.topic_name for name, _ in topics):
            self.get_logger().info(f"Data (or publisher) detected on {self.topic_name}")
            self.data_received = True
            rclpy.shutdown()
            return

        now = self.get_clock().now()
        if now > self.deadline:
            # 红色输出（终端支持时有效）
            self.get_logger().error(f"\x1b[31mNo data on {self.topic_name} within {self.timeout_sec}s, restarting...\x1b[0m")
            rclpy.shutdown()

def main():
    rclpy.init()
    topic_name = sys.argv[1] if len(sys.argv) > 1 else "/scan"
    timeout_sec = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
    node = WaitForTopicData(topic_name, timeout_sec)
    rclpy.spin(node)

if __name__ == "__main__":
    main()
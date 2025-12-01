import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import sys

class WaitForTopicData(Node):
    """
    监听指定话题，收到数据就退出。
    超时退出，用于 launch 捕获重启。
    """
    def __init__(self, topic_name, timeout_sec):
        super().__init__(f"wait_for_{topic_name.replace('/', '_')}")
        self.topic_name = topic_name
        self.timeout_sec = timeout_sec
        self.data_received = False
        self.deadline = self.get_clock().now() + rclpy.time.Duration(seconds=timeout_sec)

        # AnyMsg 可以订阅任意消息类型
        qos = QoSProfile(depth=10)
        self.create_subscription(
            msg_type=rclpy.msg.AnyMsg,
            topic=topic_name,
            callback=self.callback,
            qos_profile=qos
        )

        self.timer = self.create_timer(0.1, self.check_timeout)

    def callback(self, msg):
        self.get_logger().info(f"Data received on {self.topic_name}")
        self.data_received = True
        rclpy.shutdown()

    def check_timeout(self):
        if self.data_received:
            return
        now = self.get_clock().now()
        if now > self.deadline:
            self.get_logger().error(f"No data on {self.topic_name} within {self.timeout_sec}s, restarting...")
            rclpy.shutdown()

def main():
    rclpy.init()
    topic_name = sys.argv[1] if len(sys.argv) > 1 else "/scan"
    timeout_sec = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
    node = WaitForTopicData(topic_name, timeout_sec)
    rclpy.spin(node)

if __name__ == "__main__":
    main()

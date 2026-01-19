#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import sys
import time

class TFMonitor(Node):
    def __init__(self):
        super().__init__('tf_monitor_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 定义需要检测的链条关系 [(target, source), ...]
        self.required_transforms = [
            ('map', 'odom'),
            ('odom', 'base_link'),
            ('base_link', 'livox_frame')
        ]
        
        self.timer = self.create_timer(1.0, self.check_transforms)
        self.get_logger().info("TF Monitor started. Waiting for TF chain: map -> odom -> base_link -> livox_frame")

    def check_transforms(self):
        all_ok = True
        for target, source in self.required_transforms:
            if not self.tf_buffer.can_transform(target, source, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0)):
                self.get_logger().warn(f"Waiting for transform: {target} <- {source}")
                all_ok = False
                break 
        
        if all_ok:
            self.get_logger().info("TF完整，nav启动条件满足")
            # 退出当前节点，返回成功状态码 0
            sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = TFMonitor()
    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.shutdown()
    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()
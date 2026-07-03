#!/usr/bin/env python3
"""Relay target odom + 2Hz keep-alive for BT restart."""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

class TargetRelay(Node):
    def __init__(self):
        super().__init__("target_relay")
        self.pose_pub = self.create_publisher(PoseStamped, "/detected_target_pose", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/target_goal", 10)
        self.odom_sub = self.create_subscription(Odometry, "/target/odom", self.odom_callback, 10)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._latest_pose = None
        self.create_timer(0.5, self._keep_alive)
        self.get_logger().info("TargetRelay started")

    def odom_callback(self, msg: Odometry):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose = msg.pose.pose
        self.pose_pub.publish(pose)
        self.goal_pub.publish(pose)
        self._latest_pose = pose

    def _keep_alive(self):
        if self._latest_pose is None:
            return
        if not self.nav_client.wait_for_server(timeout_sec=0.2):
            return
        goal = NavigateToPose.Goal()
        goal.pose = self._latest_pose
        self.nav_client.send_goal_async(goal)

def main():
    rclpy.init()
    rclpy.spin(TargetRelay())
    rclpy.shutdown()
if __name__ == "__main__":
    main()

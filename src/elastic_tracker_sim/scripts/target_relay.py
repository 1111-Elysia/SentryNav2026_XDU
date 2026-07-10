#!/usr/bin/env python3
"""Relay target odom → planner / goal topics + continuous BT keep-alive."""
import math
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
        self.odom_sub = self.create_subscription(
            Odometry, "/target/odom", self.odom_callback, 10
        )
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._latest_pose = None
        self._last_sent_pose = None          # avoid duplicate goal spam
        self._goal_sent = False
        self._consecutive_failures = 0
        # Fire every 2 s: fast retry before first goal, safety-net keep-alive after.
        self.create_timer(2.0, self._try_send_goal)
        self.get_logger().info("TargetRelay started")

    def odom_callback(self, msg: Odometry):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose = msg.pose.pose
        self.pose_pub.publish(pose)
        self.goal_pub.publish(pose)
        self._latest_pose = pose

    # ------------------------------------------------------------------
    def _try_send_goal(self):
        if self._latest_pose is None:
            return
        if not self.nav_client.wait_for_server(timeout_sec=0.3):
            self._consecutive_failures += 1
            if self._consecutive_failures <= 3:
                self.get_logger().info("Waiting for nav2 action server…")
            return
        self._consecutive_failures = 0

        # Always send the very first goal.
        if not self._goal_sent:
            self._send_nav_goal(self._latest_pose)
            return

        # After the first goal: only resend when the target has moved far
        # enough, or a long heartbeat interval has passed.  This keeps the
        # BT alive without flooding the navigator with identical goals.
        now = self.get_clock().now()
        dx = (self._latest_pose.pose.position.x -
              self._last_sent_pose.pose.position.x)
        dy = (self._latest_pose.pose.position.y -
              self._last_sent_pose.pose.position.y)
        dist = math.hypot(dx, dy)

        elapsed = (now.nanoseconds - self._last_sent_time) * 1e-9
        heartbeat = 10.0   # always resend at least every 10 s

        if dist > 0.3 or elapsed > heartbeat:
            self._send_nav_goal(self._latest_pose)

    # ------------------------------------------------------------------
    def _send_nav_goal(self, pose: PoseStamped):
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.nav_client.send_goal_async(goal)
        self._goal_sent = True
        self._last_sent_pose = pose
        self._last_sent_time = self.get_clock().now().nanoseconds
        self.get_logger().debug(
            f"Sent nav goal: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})"
        )


def main():
    rclpy.init()
    rclpy.spin(TargetRelay())
    rclpy.shutdown()


if __name__ == "__main__":
    main()

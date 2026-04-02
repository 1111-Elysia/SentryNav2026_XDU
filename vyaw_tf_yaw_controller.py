#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException

from geometry_msgs.msg import Twist


class VyawTfYawController(Node):
    def __init__(self) -> None:
        super().__init__("vyaw_tf_yaw_controller")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        # 方向相关参数全部使用角度制
        self.declare_parameter("target_yaw_deg", 0.0)
        self.declare_parameter("yaw_tolerance_deg", 3.0)

        # 旋转速度约束（绝对值）
        self.declare_parameter("min_abs_vyaw", 0.15)
        self.declare_parameter("max_abs_vyaw", 1.0)

        # 简单比例控制增益（基于弧度误差）
        self.declare_parameter("k_p", 1.2)

        self.declare_parameter("control_frequency", 30.0)
        self.declare_parameter("stop_and_exit_on_success", False)

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.target_yaw_deg = float(self.get_parameter("target_yaw_deg").value)
        self.yaw_tolerance_deg = abs(float(self.get_parameter("yaw_tolerance_deg").value))

        self.min_abs_vyaw = abs(float(self.get_parameter("min_abs_vyaw").value))
        self.max_abs_vyaw = abs(float(self.get_parameter("max_abs_vyaw").value))
        self.k_p = float(self.get_parameter("k_p").value)

        self.control_frequency = float(self.get_parameter("control_frequency").value)
        self.stop_and_exit = bool(self.get_parameter("stop_and_exit_on_success").value)

        if self.max_abs_vyaw < self.min_abs_vyaw:
            self.get_logger().warn(
                "max_abs_vyaw < min_abs_vyaw, swapping them automatically"
            )
            self.min_abs_vyaw, self.max_abs_vyaw = self.max_abs_vyaw, self.min_abs_vyaw

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        period = 1.0 / max(self.control_frequency, 1.0)
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            "Started. target_yaw_deg=%.2f, tolerance=%.2f deg, vyaw[min,max]=[%.3f, %.3f]"
            % (
                self.target_yaw_deg,
                self.yaw_tolerance_deg,
                self.min_abs_vyaw,
                self.max_abs_vyaw,
            )
        )

    @staticmethod
    def _normalize_angle_rad(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        # yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _publish_vyaw(self, value: float) -> None:
        msg = Twist()
        msg.angular.z = float(value)
        self.cmd_vel_pub.publish(msg)

    def _lookup_current_yaw_rad(self):
        trans = self.tf_buffer.lookup_transform(
            self.map_frame,
            self.base_frame,
            Time(),
            timeout=Duration(seconds=0.05),
        )
        q = trans.transform.rotation
        return self._yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def _on_timer(self) -> None:
        try:
            current_yaw_rad = self._lookup_current_yaw_rad()
        except TransformException as ex:
            self.get_logger().warn(f"TF lookup failed ({self.map_frame}->{self.base_frame}): {ex}")
            self._publish_vyaw(0.0)
            return

        target_yaw_rad = math.radians(self.target_yaw_deg)
        err_rad = self._normalize_angle_rad(target_yaw_rad - current_yaw_rad)
        err_deg = math.degrees(err_rad)

        if abs(err_deg) <= self.yaw_tolerance_deg:
            self._publish_vyaw(0.0)
            self.get_logger().info(
                "Reached target yaw. current=%.2f deg, target=%.2f deg, err=%.2f deg"
                % (math.degrees(current_yaw_rad), self.target_yaw_deg, err_deg)
            )
            if self.stop_and_exit:
                rclpy.shutdown()
            return

        raw_cmd = self.k_p * abs(err_rad)
        cmd_abs = min(self.max_abs_vyaw, max(self.min_abs_vyaw, raw_cmd))
        cmd = cmd_abs if err_rad > 0.0 else -cmd_abs

        self._publish_vyaw(cmd)


def main() -> None:
    rclpy.init()
    node = VyawTfYawController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_vyaw(0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

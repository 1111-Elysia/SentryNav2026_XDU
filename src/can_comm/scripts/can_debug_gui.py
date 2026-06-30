#!/usr/bin/env python3
"""CAN 调试 GUI — 发送控制 + 实时显示收发数据"""
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sentry_msgs.msg import Vw, ScanMode, ArmorPresence
from std_msgs.msg import Bool, Float32, Int32

from threading import Lock


class CanDebugGui(Node):
    def __init__(self):
        super().__init__("can_debug_gui")

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.vw_pub = self.create_publisher(Vw, "/vw", 10)
        self.scan_pub = self.create_publisher(ScanMode, "/scan_mod_type", 10)
        self.nljg_pub = self.create_publisher(Bool, "/auto_shoot_type", 10)
        self.outpost_pub = self.create_publisher(Bool, "/outpost_mode_type", 10)
        self.armor_pub = self.create_publisher(ArmorPresence, "/detector/armor_presence", 10)
        self.yaw_ctrl_pub = self.create_publisher(Int32, "/yaw_controller", 10)
        self.target_yaw_override_pub = self.create_publisher(Float32, "/target_yaw", 10)

        self.tx_target_yaw = 0.0
        self.create_subscription(Float32, "/target_yaw", self._on_yaw_tx, 10)

        self.rx_yaw = 0.0
        self.rx_pitch = 0.0
        self.rx_dist = 0.0
        self.rx_lock = Lock()

        self.create_subscription(Float32, "/target/yaw", self._on("yaw"), 10)
        self.create_subscription(Float32, "/target/pitch", self._on("pitch"), 10)
        self.create_subscription(Float32, "/target/distance", self._on("dist"), 10)

        self.tx_lock = Lock()
        self.tx_vx = 0.0
        self.tx_vy = 0.0
        self.tx_vw = 0.0
        self.tx_scan = False
        self.tx_nljg = False
        self.tx_outpost = False
        self.tx_left = False
        self.tx_behind = False
        self.tx_right = False

        self.pub_timer = self.create_timer(0.05, self._pub_tick)

    def _on_yaw_tx(self, m):
        with self.tx_lock:
            self.tx_target_yaw = m.data

    def _on(self, attr):
        def cb(m):
            with self.rx_lock:
                setattr(self, f"rx_{attr}", m.data)
        return cb

    def _pub_tick(self):
        t = Twist()
        s = ScanMode()
        v = Vw()
        b_nljg = Bool()
        b_out = Bool()
        a = ArmorPresence()
        with self.tx_lock:
            t.linear.x = self.tx_vx
            t.linear.y = self.tx_vy
            v.vw = self.tx_vw
            s.scan_mod_type = self.tx_scan
            b_nljg.data = self.tx_nljg
            b_out.data = self.tx_outpost
            a.left = self.tx_left
            a.behind = self.tx_behind
            a.right = self.tx_right
        self.cmd_vel_pub.publish(t)
        self.vw_pub.publish(v)
        self.scan_pub.publish(s)
        self.nljg_pub.publish(b_nljg)
        self.outpost_pub.publish(b_out)
        self.armor_pub.publish(a)


def build_gui(node: CanDebugGui):
    root = tk.Tk()
    root.title("CAN Debug — can_send / can_receive")
    root.geometry("720x540")
    root.resizable(False, False)

    ctrl = ttk.LabelFrame(root, text="发送控制", padding=10)
    ctrl.place(x=10, y=10, width=340, height=520)

    # ── 滑条 ──
    def _make_slider_cb(attr, disp):
        def cb(val_str):
            v = float(val_str) / 100.0
            disp.set(f"{v:.2f}")
            with node.tx_lock:
                setattr(node, attr, v)
        return cb

    for row, (label, attr, rng) in enumerate([
        ("vx", "tx_vx", 100), ("vy", "tx_vy", 100), ("vw", "tx_vw", 100),
    ]):
        ttk.Label(ctrl, text=label).grid(row=row, column=0, sticky="e", padx=(0, 5))
        disp = tk.StringVar(value="0.00")
        ttk.Label(ctrl, textvariable=disp, width=7).grid(row=row, column=1)
        s = ttk.Scale(ctrl, from_=-rng, to=rng, command=_make_slider_cb(attr, disp))
        s.set(0)
        s.grid(row=row, column=2, padx=5)

    # target_yaw 滑条: 范围 [-180, 180]，直接发布到 /target_yaw
    ty_row = 3
    ttk.Separator(ctrl, orient="horizontal").grid(row=ty_row, columnspan=3, sticky="ew", pady=10)
    ttk.Label(ctrl, text="target_yaw").grid(row=ty_row + 1, column=0, sticky="e", padx=(0, 5))
    ty_disp = tk.StringVar(value="0.0")
    ttk.Label(ctrl, textvariable=ty_disp, width=7).grid(row=ty_row + 1, column=1)

    def _on_target_yaw_slider(val_str):
        v = float(val_str) / 10.0      # Scale int → deg, 精度 0.1°
        ty_disp.set(f"{v:.1f}")
        node.target_yaw_override_pub.publish(Float32(data=v))

    ty_s = ttk.Scale(ctrl, from_=-1800, to=1800, command=_on_target_yaw_slider)
    ty_s.set(0)
    ty_s.grid(row=ty_row + 1, column=2, padx=5)

    ttk.Separator(ctrl, orient="horizontal").grid(row=ty_row + 2, columnspan=3, sticky="ew", pady=10)

    # ── 勾选框 ──
    def _set_toggle(attr):
        def cb():
            with node.tx_lock:
                v = getattr(node, f"{attr}_var", None)
                setattr(node, attr, v.get() if v else False)
        return cb

    toggles = []
    for row, (label, attr) in enumerate([
        ("scan_mode", "tx_scan"),
        ("NLJG_mode (打符)", "tx_nljg"),
        ("outpost_mode", "tx_outpost"),
    ], start=6):
        var = tk.BooleanVar()
        setattr(node, f"{attr}_var", var)
        var.trace_add("write", lambda *a, a2=attr: setattr(
            node, a2, getattr(node, f"{a2}_var").get()))
        ttk.Checkbutton(ctrl, text=label, variable=var).grid(
            row=row, column=0, columnspan=3, sticky="w", padx=50)
        toggles.append((attr, var))

    ttk.Separator(ctrl, orient="horizontal").grid(row=9, columnspan=3, sticky="ew", pady=10)

    for row, (label, attr) in enumerate([
        ("armor_left", "tx_left"),
        ("armor_behind", "tx_behind"),
        ("armor_right", "tx_right"),
    ], start=10):
        var = tk.BooleanVar()
        setattr(node, f"{attr}_var", var)
        var.trace_add("write", lambda *a, a2=attr: setattr(
            node, a2, getattr(node, f"{a2}_var").get()))
        ttk.Checkbutton(ctrl, text=label, variable=var).grid(
            row=row, column=0, columnspan=3, sticky="w", padx=50)

    # ── yaw controller 触发按钮 ──
    btn_row = 13
    ttk.Separator(ctrl, orient="horizontal").grid(
        row=btn_row, columnspan=3, sticky="ew", pady=10)
    ttk.Label(ctrl, text="yaw_controller:").grid(
        row=btn_row + 1, column=0, sticky="e", padx=(0, 5))
    btn_frame = ttk.Frame(ctrl)
    btn_frame.grid(row=btn_row + 1, column=1, columnspan=2, sticky="w")
    ttk.Button(btn_frame, text="NLJG (0)", width=10,
               command=lambda: node.yaw_ctrl_pub.publish(Int32(data=0))).pack(side="left", padx=2)
    ttk.Button(btn_frame, text="Outpost (1)", width=10,
               command=lambda: node.yaw_ctrl_pub.publish(Int32(data=1))).pack(side="left", padx=2)

    # ── 右侧: 显示 ──
    disp = ttk.LabelFrame(root, text="实时数据", padding=10)
    disp.place(x=360, y=10, width=350, height=520)

    ttk.Label(disp, text="── 发送到 CAN ──", font=("", 10, "bold")).grid(
        row=0, column=0, columnspan=2, sticky="w", pady=(0, 5))

    tx_labels = {}
    tx_items = [
        ("tx_vx", "vx"), ("tx_vy", "vy"), ("tx_vw", "vw"),
        ("tx_target_yaw", "target_yaw"),
        ("tx_scan", "scan_mode"), ("tx_nljg", "NLJG"), ("tx_outpost", "outpost"),
        ("tx_left", "armor_left"), ("tx_behind", "armor_behind"), ("tx_right", "armor_right"),
    ]
    for i, (attr, label) in enumerate(tx_items):
        ttk.Label(disp, text=f"{label}:").grid(row=i + 1, column=0, sticky="e", padx=(0, 5))
        v = tk.StringVar(value="-")
        ttk.Label(disp, textvariable=v, width=8, anchor="w").grid(row=i + 1, column=1, sticky="w")
        tx_labels[attr] = v

    row_rx = len(tx_items) + 2
    ttk.Separator(disp, orient="horizontal").grid(row=row_rx, columnspan=2, sticky="ew", pady=10)
    ttk.Label(disp, text="── 从 CAN 接收 ──", font=("", 10, "bold")).grid(
        row=row_rx + 1, column=0, columnspan=2, sticky="w", pady=(0, 5))

    rx_labels = {}
    for i, (attr, label) in enumerate([
        ("rx_yaw", "target_yaw (°)"), ("rx_pitch", "target_pitch (°)"), ("rx_dist", "distance (m)"),
    ]):
        ttk.Label(disp, text=f"{label}:").grid(
            row=row_rx + 2 + i, column=0, sticky="e", padx=(0, 5))
        v = tk.StringVar(value="-")
        ttk.Label(disp, textvariable=v, width=8, anchor="w").grid(
            row=row_rx + 2 + i, column=1, sticky="w")
        rx_labels[attr] = v

    def refresh():
        with node.tx_lock:
            tx = {a: getattr(node, a) for a, _ in tx_items}
        for attr, val in tx.items():
            if isinstance(val, bool):
                tx_labels[attr].set("ON" if val else "OFF")
            else:
                tx_labels[attr].set(f"{val:.2f}")

        with node.rx_lock:
            rx = {a: getattr(node, a) for a in rx_labels}
        for attr, val in rx.items():
            rx_labels[attr].set(f"{val:.2f}")

        root.after(200, refresh)

    root.after(200, refresh)
    root.mainloop()


def main():
    rclpy.init()
    node = CanDebugGui()
    import threading
    t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    t.start()
    build_gui(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
地面机器人位置 GUI 模拟器
模拟我方机器人位置数据，发布到 /ground_pos_sim/ground_robot_position
"""

import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from rm_referee_msgs.msg import GroundRobotPosition


class GroundPosSimulator(Node):
    """ROS2 节点：发布模拟的地面机器人位置"""

    def __init__(self):
        super().__init__('ground_pos_simulator')
        self.publisher = self.create_publisher(
            GroundRobotPosition,
            '/ground_pos_sim/ground_robot_position',
            10
        )
        self.get_logger().info('GroundPosSimulator node started')

    def publish(self, hero_x, hero_y, engineer_x, engineer_y,
                standard_3_x, standard_3_y, standard_4_x, standard_4_y,
                reserved, reserved_2):
        msg = GroundRobotPosition()
        msg.hero_x = hero_x
        msg.hero_y = hero_y
        msg.engineer_x = engineer_x
        msg.engineer_y = engineer_y
        msg.standard_3_x = standard_3_x
        msg.standard_3_y = standard_3_y
        msg.standard_4_x = standard_4_x
        msg.standard_4_y = standard_4_y
        msg.reserved = reserved
        msg.reserved_2 = reserved_2
        self.publisher.publish(msg)
        self.get_logger().debug('Published ground robot position')


class SimulatorGUI:
    """tkinter GUI 界面"""

    def __init__(self, node: GroundPosSimulator):
        self.node = node
        self.root = tk.Tk()
        self.root.title("地面机器人位置模拟器")
        self.root.resizable(False, False)

        # 主框架
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # 标题
        title = ttk.Label(main_frame, text="我方机器人位置模拟", font=('Arial', 14, 'bold'))
        title.grid(row=0, column=0, columnspan=2, pady=(0, 10))

        # 机器人位置输入
        self.entries = {}
        robots = [
            ("英雄 (Hero)", "hero"),
            ("工程 (Engineer)", "engineer"),
            ("步兵3号 (Standard 3)", "standard_3"),
            ("步兵4号 (Standard 4)", "standard_4"),
        ]

        row = 1
        for label, key in robots:
            ttk.Label(main_frame, text=label, font=('Arial', 10, 'bold')).grid(
                row=row, column=0, columnspan=2, sticky=tk.W, pady=(8, 2))

            row += 1
            ttk.Label(main_frame, text="X:").grid(row=row, column=0, sticky=tk.E, padx=(20, 5))
            x_entry = ttk.Entry(main_frame, width=12)
            x_entry.grid(row=row, column=1, sticky=tk.W)
            x_entry.insert(0, "0.0")
            self.entries[f"{key}_x"] = x_entry

            row += 1
            ttk.Label(main_frame, text="Y:").grid(row=row, column=0, sticky=tk.E, padx=(20, 5))
            y_entry = ttk.Entry(main_frame, width=12)
            y_entry.grid(row=row, column=1, sticky=tk.W)
            y_entry.insert(0, "0.0")
            self.entries[f"{key}_y"] = y_entry
            row += 1

        # 预留字段
        ttk.Label(main_frame, text="预留字段", font=('Arial', 10, 'bold')).grid(
            row=row, column=0, columnspan=2, sticky=tk.W, pady=(8, 2))
        row += 1
        for i, label in enumerate(["reserved", "reserved_2"]):
            ttk.Label(main_frame, text=f"{label}:").grid(
                row=row, column=0, sticky=tk.E, padx=(20, 5))
            entry = ttk.Entry(main_frame, width=12)
            entry.grid(row=row, column=1, sticky=tk.W)
            entry.insert(0, "0.0")
            self.entries[label] = entry
            row += 1

        # 操作按钮
        btn_frame = ttk.Frame(main_frame)
        btn_frame.grid(row=row, column=0, columnspan=2, pady=(15, 0))

        self.send_btn = ttk.Button(btn_frame, text="发送一次", command=self.send_once)
        self.send_btn.grid(row=0, column=0, padx=5)

        self.auto_btn = ttk.Button(btn_frame, text="开始自动发送", command=self.toggle_auto)
        self.auto_btn.grid(row=0, column=1, padx=5)

        # 频率选择
        ttk.Label(btn_frame, text="频率(Hz):").grid(row=1, column=0, sticky=tk.E, padx=5, pady=(10, 0))
        self.freq_var = tk.StringVar(value="10")
        freq_spin = ttk.Spinbox(btn_frame, from_=1, to=50, textvariable=self.freq_var, width=8)
        freq_spin.grid(row=1, column=1, sticky=tk.W, pady=(10, 0))

        # 状态栏
        self.status_var = tk.StringVar(value="就绪 - 手动发送")
        status_label = ttk.Label(main_frame, textvariable=self.status_var,
                                 font=('Arial', 9), foreground='gray')
        status_label.grid(row=row + 3, column=0, columnspan=2, pady=(10, 0))

        self.auto_running = False
        self.auto_job = None

    def get_values(self):
        """从 GUI 读取所有值"""
        try:
            vals = {}
            for key, entry in self.entries.items():
                vals[key] = float(entry.get())
            return vals
        except ValueError:
            return None

    def send_once(self):
        """手动发送一次"""
        vals = self.get_values()
        if vals is None:
            self.status_var.set("错误：请输入有效数字")
            return
        self.node.publish(
            vals["hero_x"], vals["hero_y"],
            vals["engineer_x"], vals["engineer_y"],
            vals["standard_3_x"], vals["standard_3_y"],
            vals["standard_4_x"], vals["standard_4_y"],
            vals["reserved"], vals["reserved_2"]
        )
        self.status_var.set("已发送一次")

    def toggle_auto(self):
        """切换自动发送"""
        if self.auto_running:
            self.auto_running = False
            if self.auto_job:
                self.root.after_cancel(self.auto_job)
                self.auto_job = None
            self.auto_btn.config(text="开始自动发送")
            self.send_btn.config(state="normal")
            self.status_var.set("已停止自动发送")
        else:
            self.auto_running = True
            self.auto_btn.config(text="停止自动发送")
            self.send_btn.config(state="disabled")
            self.auto_send()

    def auto_send(self):
        """自动发送循环"""
        if not self.auto_running:
            return
        vals = self.get_values()
        if vals is not None:
            self.node.publish(
                vals["hero_x"], vals["hero_y"],
                vals["engineer_x"], vals["engineer_y"],
                vals["standard_3_x"], vals["standard_3_y"],
                vals["standard_4_x"], vals["standard_4_y"],
                vals["reserved"], vals["reserved_2"]
            )
            self.status_var.set("自动发送中...")
        else:
            self.status_var.set("错误：请输入有效数字")

        try:
            freq = float(self.freq_var.get())
            interval = int(1000.0 / max(freq, 1.0))
        except ValueError:
            interval = 100

        self.auto_job = self.root.after(interval, self.auto_send)

    def run(self):
        """启动 GUI 主循环"""

        def ros_spin():
            """在 tkinter 事件循环中轮询 ROS"""
            rclpy.spin_once(self.node, timeout_sec=0.001)
            self.root.after(10, ros_spin)

        self.root.after(10, ros_spin)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.mainloop()

    def on_close(self):
        """关闭窗口"""
        self.auto_running = False
        self.root.destroy()


def main():
    rclpy.init()
    node = GroundPosSimulator()
    gui = SimulatorGUI(node)
    gui.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

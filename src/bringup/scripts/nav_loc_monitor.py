#!/usr/bin/env python3
"""
Nav/Loc Monitor
监控 Nav2 的 Lifecycle 状态 (Navigation 和 Localization)
如果发现管理器或关键节点未处于 Active 状态超过指定时间，则触发重启。
"""

import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState
from std_msgs.msg import Bool
import time

class NavLocMonitor(Node):
    def __init__(self):
        super().__init__('nav_loc_monitor')
        
        # =======================
        # 配置参数
        # =======================
        # 需要监控的 Lifecycle 节点名称列表
        # 通常监控两个 Manager 即可，因为如果 Manager 挂了或者不活跃，子节点也就没用了
        self.target_nodes = [
            '/lifecycle_manager_navigation',
            '/lifecycle_manager_localization'
        ]
        
        # 允许的启动缓冲时间 (秒) - 系统刚启动时不要因为没 Active 就重启
        self.startup_grace_period = 40.0 
        
        # 故障确认时间 (秒) - 连续多少秒检测到非 Active 才重启
        self.failure_timeout = 5.0
        
        # 检查频率 (Hz)
        self.check_frequency = 1.0

        # =======================
        # 内部状态
        # =======================
        self.node_states = {node: False for node in self.target_nodes} # True=Active, False=Inactive
        self.start_time = time.time()
        self.failure_start_time = None
        self.restart_triggered = False

        # 创建 Service Clients
        # [修复] 变量名 clients 与 rclpy.Node.clients 属性冲突，修改为 lifecycle_clients
        self.lifecycle_clients = {}
        for node_name in self.target_nodes:
            srv_name = f'{node_name}/get_state'
            self.lifecycle_clients[node_name] = self.create_client(GetState, srv_name)
            
        # 重启信号发布者
        self.restart_pub = self.create_publisher(Bool, '/system/restart_request', 10)

        # 定时器
        self.timer = self.create_timer(1.0 / self.check_frequency, self.check_lifecycle_states)
        
        self.get_logger().info(f"Nav/Loc Monitor 启动。监控节点: {self.target_nodes}")
        self.get_logger().info(f"启动宽限期: {self.startup_grace_period}s")

    def check_lifecycle_states(self):
        # 0. 如果已经触发重启，不再执行
        if self.restart_triggered:
            self.trigger_restart() # 持续发送，确保收到
            return

        # 1. 检查是否还在启动宽限期内
        if time.time() - self.start_time < self.startup_grace_period:
            return

        # 2. 异步查询每个节点的状态
        # [修复] 更新遍历的字典名称
        for node_name, client in self.lifecycle_clients.items():
            if not client.service_is_ready():
                self.get_logger().warn(f"服务不可用: {node_name}/get_state (可能节点已挂)")
                self.node_states[node_name] = False
                continue
            
            # 发送异步请求
            req = GetState.Request()
            future = client.call_async(req)
            # 使用回调处理结果，也可以在这里简单地不做阻塞等待，
            # 为简单起见，我们增加一个 callback 处理函数
            future.add_done_callback(lambda future, name=node_name: self.handle_service_response(future, name))

        # 3. 评估整体健康状况 (在下一轮循环或回调更新后生效)
        # 这种异步模式会导致当前帧判断的是上一帧的结果，对于监控来说是可以接受的
        self.evaluate_health()

    def handle_service_response(self, future, node_name):
        try:
            response = future.result()
            # ID 3 = PRIMARY_STATE_ACTIVE
            is_active = (response.current_state.id == 3)
            self.node_states[node_name] = is_active
            # if not is_active:
            #     # 调试日志，平时可以注释掉减少刷屏
            #     pass 
        except Exception as e:
            self.get_logger().error(f"调用服务 {node_name} 失败: {e}")
            self.node_states[node_name] = False

    def evaluate_health(self):
        all_active = all(self.node_states.values())

        if all_active:
            # 如果健康，重置故障计时器
            if self.failure_start_time is not None:
                self.get_logger().info("Nav/Loc 系统恢复 Active 状态。")
            self.failure_start_time = None
        else:
            # 如果不健康
            if self.failure_start_time is None:
                self.failure_start_time = time.time()
                self.get_logger().warn("Nav/Loc 系统检测到非 Active 状态，开始故障计时...")
                # 打印具体哪个挂了
                for name, active in self.node_states.items():
                    if not active:
                        self.get_logger().warn(f"  -> {name} is INACTIVE / UNREACHABLE")

            elapsed = time.time() - self.failure_start_time
            if elapsed > self.failure_timeout:
                self.get_logger().error(f"Nav/Loc 系统失效持续 {elapsed:.1f}s (> {self.failure_timeout}s). 触发重启。")
                self.restart_triggered = True
                self.trigger_restart()

    def trigger_restart(self):
        """发送重启信号"""
        msg = Bool()
        msg.data = True
        self.restart_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NavLocMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
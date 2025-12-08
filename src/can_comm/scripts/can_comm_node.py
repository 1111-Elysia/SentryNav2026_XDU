#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
import yaml
import pyreferee
import importlib

# 标准 ROS2 消息包
import std_msgs.msg as std
# 自定义消息包
import sentry_msgs.msg as sm

def get_from_path(obj, path):
    """支持类似 'custom_robot_data[0]' 的路径获取值"""
    try:
        parts = path.replace(']', '').split('[')
        val = obj
        for p in parts:
            if p.isdigit():
                val = val[int(p)]
            else:
                val = getattr(val, p)
        return val
    except Exception:
        return None

class CanCommNode(Node):
    def __init__(self, yaml_path=None):
        super().__init__('can_comm_node')

        # 自动生成 YAML 路径
        if yaml_path is None:
            default_yaml = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                'config/can_comm_params.yaml'
            )
            yaml_path = os.environ.get('CAN_COMM_YAML', default_yaml)

        if not os.path.exists(yaml_path):
            self.get_logger().error(f'YAML file not found: {yaml_path}')
            raise FileNotFoundError(f'YAML file not found: {yaml_path}')

        # 读取 YAML 配置
        with open(yaml_path, 'r') as f:
            self.cfg = yaml.safe_load(f)

        # cmd_id -> {MsgClass, publisher, field_map}
        self.can_map = {}
        self.init_publishers()

        # 初始化 Referee
        self.ref = pyreferee.Referee()
        self.ref.attach_callback(self.on_referee_data)

        self.get_logger().info(f'Python CAN comm node started using {yaml_path}.')

    def init_publishers(self):
        for item in self.cfg['can_to_ros']:
            raw = item['cmd_id']
            if isinstance(raw, str):
                cmd_id = int(raw, 0)  # 支持 "0x0001" 形式
            else:
                cmd_id = int(raw)      # 支持整数
            msg_type_str = item['msg_type']  # 仅类名，例如 MyUInt16
            topic = item['topic']
            field_map = item.get('field_map', {})

            # 优先从自定义消息包获取
            MsgClass = getattr(sm, msg_type_str, None)
            # 若自定义消息包没有，再尝试标准消息包
            if MsgClass is None:
                MsgClass = getattr(std, msg_type_str, None)

            if MsgClass is None:
                self.get_logger().error(f'Message class {msg_type_str} not found in sentry_msgs.msg or std_msgs.msg')
                continue

            pub = self.create_publisher(MsgClass, topic, 10)
            self.can_map[cmd_id] = {'MsgClass': MsgClass, 'publisher': pub, 'field_map': field_map}

    def on_referee_data(self, cmd_id, seq):
        info = self.can_map.get(cmd_id)
        if info is None:
            return

        MsgClass = info['MsgClass']
        pub = info['publisher']
        field_map = info['field_map']
        data_obj = self.ref.get_data()

        # 创建消息实例
        msg = MsgClass()

        # 根据 field_map 动态赋值
        for field, path in field_map.items():
            value = get_from_path(data_obj, path)
            if value is not None:
                setattr(msg, field, value)

        pub.publish(msg)
        self.get_logger().debug(f'Published {MsgClass.__name__} for cmd_id=0x{cmd_id:04X}')

def main(args=None):
    rclpy.init(args=args)
    try:
        node = CanCommNode()
        rclpy.spin(node)
    except FileNotFoundError as e:
        print(e)
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()

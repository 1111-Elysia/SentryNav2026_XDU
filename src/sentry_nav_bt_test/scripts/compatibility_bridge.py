#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# 1. 导入消息类型
try:

    import rm2_referee_msgs.msg as old_msgs
    import rm_referee_msgs.msg as new_msgs
except ImportError as e:
    print(f"Error importing messages: {e}")
    print("请确保 rm2_referee_msgs 和 rm_referee_msgs 都在当前工作空间或环境中")
    exit(1)

# 2. 通用字段复制函数 
def transfer_fields(src_msg, dst_msg, logger=None):
    """
    自动将 src_msg 的字段值复制到 dst_msg 中。
    基于字段名称匹配。
    """
    # 获取目标消息的所有字段槽 (slots)
    # 比如 ['robot_id', 'current_hp', ...]
    if not hasattr(dst_msg, 'get_fields_and_field_types'):
        return dst_msg
        
    fields = dst_msg.get_fields_and_field_types().keys()

    for field_name in fields:
        # 跳过 header，通常我们会手动处理 header 以更新时间戳
        if field_name == 'header':
            continue

        # 如果源消息也有这个字段，就复制过去
        if hasattr(src_msg, field_name):
            try:
                val = getattr(src_msg, field_name)
                setattr(dst_msg, field_name, val)
            except Exception as e:
                if logger:
                    logger.warn(f"字段 '{field_name}' 类型转换失败: {e}")
        else:
            # 如果旧消息里没有这个字段 (新协议新增的)，跳过或给默认值
            pass
            
    return dst_msg

# 3. 桥接节点类
class RefereeCompatibilityBridge(Node):
    def __init__(self):
        super().__init__('referee_compatibility_bridge')
        # 阵营参数（red / blue）
        self.declare_parameter('team_color', 'red')
        self.team_color = self.get_parameter('team_color').value
        self.get_logger().info(f"当前阵营颜色: {self.team_color}")

        # 定义 QoS 
        self.qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        
        self.get_logger().info("正在启动旧协议(rm2) -> 新协议(rm) 桥接器...")

        # 定义需要转换的话题列表
        # 格式: (旧消息类型, 新消息类型, 话题名后缀)
        # 话题名会自动拼接到 /rm2_referee/ 和 /rm_referee/ 后面
        self.bridges = [
            (old_msgs.RobotPos,            new_msgs.RobotPos,            'robot_pos'),
            (old_msgs.SentryInfo,          new_msgs.SentryInfo,          'sentry_info'),
            (old_msgs.GameStatus,          new_msgs.GameStatus,          'game_status'),
            (old_msgs.RobotStatus,         new_msgs.RobotStatus,         'robot_status'),
            (old_msgs.GameRobotHP,         new_msgs.GameRobotHP,         'game_robot_hp'),
            (old_msgs.ProjectileAllowance, new_msgs.ProjectileAllowance, 'projectile_allowance'),
            (old_msgs.HurtData,            new_msgs.HurtData,            'hurt_data'),
            (old_msgs.EventData,           new_msgs.EventData,           'event_data'),
            (old_msgs.RfidStatus,        new_msgs.RfidStatus,          'rfid_status'),
        ]

        # 存储 subscribers 和 publishers 以防被垃圾回收
        self.subs = []
        self.pubs = []

        # 批量创建桥接
        for old_type, new_type, topic_suffix in self.bridges:
            self.create_bridge(old_type, new_type, topic_suffix)

    def create_bridge(self, old_type, new_type, topic_suffix):
        """创建一对 sub/pub"""
        old_topic = f"/rm2_referee/{topic_suffix}"
        new_topic = f"/rm_referee/{topic_suffix}"

        # 1. 创建发布者 (发给你的 BT)
        pub = self.create_publisher(new_type, new_topic, self.qos)
        self.pubs.append(pub)

        # 2. 创建回调函数 (利用闭包捕获 pub 和 new_type)
        def callback(msg_old, p=pub, t=new_type, name=topic_suffix):
            msg_new = t()

            # 特殊处理 GameRobotHP
            if name == "game_robot_hp":
                self.map_game_robot_hp(msg_old, msg_new)
            else:
                transfer_fields(msg_old, msg_new, self.get_logger())

            # header
            if hasattr(msg_new, 'header') and hasattr(msg_old, 'header'):
                msg_new.header = msg_old.header

            p.publish(msg_new)
        # 3. 创建订阅者 (监听模拟器)
        sub = self.create_subscription(old_type, old_topic, callback, self.qos)
        self.subs.append(sub)
        
        self.get_logger().info(f"已建立桥接: {old_topic} -> {new_topic}")

    def map_game_robot_hp(self, old_msg, new_msg):

        if self.team_color == "red":
            prefix = "red"
        else:
            prefix = "blue"

        new_msg.ally_1_robot_hp = getattr(old_msg, f"{prefix}_1_robot_hp")
        new_msg.ally_2_robot_hp = getattr(old_msg, f"{prefix}_2_robot_hp")
        new_msg.ally_3_robot_hp = getattr(old_msg, f"{prefix}_3_robot_hp")
        new_msg.ally_4_robot_hp = getattr(old_msg, f"{prefix}_4_robot_hp")
        new_msg.ally_7_robot_hp = getattr(old_msg, f"{prefix}_7_robot_hp")

        new_msg.ally_outpost_hp = getattr(old_msg, f"{prefix}_outpost_hp")
        new_msg.ally_base_hp = getattr(old_msg, f"{prefix}_base_hp")

        new_msg.reserved = 0
    

def main(args=None):
    rclpy.init(args=args)
    node = RefereeCompatibilityBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
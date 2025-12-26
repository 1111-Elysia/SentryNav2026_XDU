# 用法: python3 send_sentry_posture.py 1|2|3
import sys
import rclpy
from rclpy.node import Node
from rm_referee_msgs.srv import Tx

def crc8_maxim_head(hdr4):
    crc = 0xFF
    for b in hdr4:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x31) & 0xFF if (crc & 0x80) else ((crc << 1) & 0xFF)
    return crc & 0xFF

def crc16_ibm(frame):
    crc = 0xFFFF
    for b in frame:
        crc ^= (b << 8)
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else ((crc << 1) & 0xFFFF)
    return crc & 0xFFFF

def build_rm_frame(cmd_id: int, data: bytes, seq: int = 0) -> list[int]:
    data_len = 2 + len(data)
    hdr4 = [0xA5, data_len & 0xFF, (data_len >> 8) & 0xFF, seq & 0xFF]
    crc8 = crc8_maxim_head(hdr4)
    frame = hdr4 + [crc8]
    frame += [cmd_id & 0xFF, (cmd_id >> 8) & 0xFF]
    frame += list(data)
    crc16 = crc16_ibm(frame)
    frame += [crc16 & 0xFF, (crc16 >> 8) & 0xFF]
    return frame

def main():
    if len(sys.argv) != 2 or sys.argv[1] not in ('1','2','3'):
        print("用法: python3 send_sentry_posture.py 1|2|3  (1=进攻, 2=防御, 3=移动)")
        return
    posture = int(sys.argv[1])  # 1..3

    # 仅设置 bit21-22 为姿态值，其他全部清零
    sentry_cmd = (posture & 0x3) << 21
    payload = sentry_cmd.to_bytes(4, 'little')

    frame = build_rm_frame(cmd_id=0x0120, data=payload, seq=0)  # seq 可自增

    rclpy.init()
    node = Node('sentry_posture_tx')
    cli = node.create_client(Tx, '/rm_referee/tx')
    if not cli.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('服务 /rm_referee/tx 不可用')
        return
    req = Tx.Request()
    req.data = frame
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut)
    if fut.result() is not None:
        node.get_logger().info(f'发送姿态={posture}, ok={fut.result().ok}')
    else:
        node.get_logger().error('调用失败')
    rclpy.shutdown()

if __name__ == '__main__':
    main()
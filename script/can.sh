#!/bin/bash
# 文件名: setup_can_udev.sh
# 用途: 自动创建 udev 规则，使 can0/can1/can2 在启动时自动配置

RULE_FILE="/etc/udev/rules.d/99-can.rules"

# 检查是否以 root 运行
if [ "$EUID" -ne 0 ]; then
    echo "请以 root 用户运行此脚本 (sudo ./setup_can_udev.sh)"
    exit 1
fi

# 备份已有规则文件
if [ -f "$RULE_FILE" ]; then
    cp "$RULE_FILE" "${RULE_FILE}.bak_$(date +%s)"
    echo "已有规则文件已备份: ${RULE_FILE}.bak_$(date +%s)"
fi

# 写入 udev 规则
cat << EOF > "$RULE_FILE"
# 自动 bring up CAN 接口
SUBSYSTEM=="net", KERNEL=="can*", ACTION=="add", RUN+="/usr/sbin/ip link set %k type can bitrate 1000000"
SUBSYSTEM=="net", KERNEL=="can*", ACTION=="add", RUN+="/usr/sbin/ip link set %k up""
EOF

# 重新加载 udev
udevadm control --reload
udevadm trigger

echo "✅ udev 规则已创建并应用: $RULE_FILE"
echo "系统启动或插入 CAN 设备时，会自动初始化"
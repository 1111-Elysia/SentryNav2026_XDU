#!/bin/bash

# 设置话题和保存路径
TOPICS="/rosout"  # 需要录制的 ROS 话题
SAVE_DIR="/home/pgd/SentryNav2026_XDU/save"   # 保存目录
RECORD_INTERVAL=5        # 录制间隔时间（单位：秒）

# 每次录制生成一个新的时间戳
while true; do
  TIMESTAMP=$(date +'%Y%m%d_%H%M%S')
  BAG_NAME="recording_$TIMESTAMP"

  # 启动 ros2 bag 录制
  echo "Starting recording $BAG_NAME..."
  ros2 bag record $TOPICS --output $SAVE_DIR/$BAG_NAME --max-cache 100 &
  BAG_PID=$!

  # 等待一定时间，然后停止录制
  sleep $RECORD_INTERVAL

  # 通过发送 SIGINT 信号优雅地停止 ros2 bag
  echo "Stopping recording $BAG_NAME..."
  kill -SIGINT $BAG_PID

  # 等待 ros2 bag 进程结束
  wait $BAG_PID

done

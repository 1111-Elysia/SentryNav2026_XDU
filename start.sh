source ./install/setup.bash 
echo "导航，启动！"
while true; do
  echo
  ros2 launch bringup monitored_start.launch.py
  exit_code=$?

  echo "[ERROR] 导航异常重启: $exit_code"

  # Ctrl+C → 130 (SIGINT)
  if [ "$exit_code" -eq 130 ]; then
    echo "检测到 Ctrl+C，退出监控程序"
    exit 0
  fi

  echo "[ERROR] 机魂不悦，自动重启中..."
done
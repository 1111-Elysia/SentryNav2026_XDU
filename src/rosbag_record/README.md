# rosbag_record

订阅 `/rm_referee/game_status`：

- `game_progress == 3` 时启动录包
- `game_progress == 5` 时停止录包

录制的话题从参数文件的 `topics` 读取。

## 运行

推荐使用 launch（默认会加载本包内的参数文件路径）：

```bash
source install/setup.bash

ros2 launch rosbag_record rosbag_record.launch.py
```

如果想自定义参数文件：

```bash
ros2 launch rosbag_record rosbag_record.launch.py params_file:=/abs/path/to/rosbag_record.yaml
```

也可以直接 `ros2 run`，但需要显式指定参数文件：

```bash
source install/setup.bash

ros2 run rosbag_record rosbag_record_node.py \
  --ros-args --params-file src/rosbag_record/config/rosbag_record.yaml
```

## 参数

- `topics`：string 数组，要录制的话题名
- `output_dir`：bag 保存目录（必须为绝对路径，例如 `/data/bags` 或 `/tmp/rosbags`）
- `bag_prefix`：bag 名称前缀（最终为 `${bag_prefix}_YYYYmmdd_HHMMSS`）
- `start_game_progress` / `stop_game_progress`

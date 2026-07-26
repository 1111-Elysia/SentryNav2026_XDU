# sentry_nav_bt_gui

`sentry_nav_bt_gui` 是 `sentry_nav_bt` 单点导航行为树的 rqt 控制面板。它通过 ROS 2 参数服务读取和原子更新目标点、临时坐标、移动/等待姿态以及到点和等待阈值，不直接发布导航目标。

## 工作方式

```text
Simple Nav Control
        │
        ├── /sentry_nav_bt/get_parameters
        └── /sentry_nav_bt/set_parameters_atomically
                         │
                         ▼
             sentry_nav_bt
               simple_nav.xml
                         │
                         ▼
               NavigateToPose / 裁判系统
```

插件每 200 ms 检查参数服务是否可用。首次连接后读取当前配置；点击“应用配置”时，九个参数通过 `SetParametersAtomically` 一次提交。行为树节点负责校验参数并重新开始一次单点任务。

目标点列表读取自已安装的：

```text
sentry_nav_bt/config/waypoints.json
```

## 依赖

- ROS 2 Humble
- `rqt_gui`、`rqt_gui_py`
- `python_qt_binding`
- `rclpy`、`rcl_interfaces`
- `sentry_nav_bt`

## 编译

在工作空间根目录执行：

```bash
colcon build --packages-up-to sentry_nav_bt_gui
source install/setup.bash
```

源码更新后需要重新编译并重新 `source install/setup.bash`，rqt 才会加载安装空间中的新版本。

## 启动

同时启动单点导航行为树和控制面板：

```bash
ros2 launch sentry_nav_bt_gui simple_nav_with_gui.launch.py
```

只启动控制面板：

```bash
ros2 launch sentry_nav_bt_gui gui.launch.py
```

后一种方式要求 `/sentry_nav_bt` 已由其他终端启动，例如：

```bash
ros2 launch sentry_nav_bt simple_nav.launch.py
```

也可以从 rqt 菜单打开：

```text
Plugins > Sentry Navigation > Simple Nav Control
```

## 控件说明

| 控件 | 含义 |
|---|---|
| 点位 | 从 `waypoints.json` 选择命名路径点 |
| 使用临时坐标 | 启用手动 X、Y、Yaw，不修改路径点文件 |
| 移动姿态 | 前往目标点期间的哨兵姿态，范围 1～6 |
| 等待姿态 | 到点后等待期间的哨兵姿态，范围 1～6 |
| 到点距离 | 本地到点判定阈值，单位 m |
| 时间阈值 | 到点后的持续等待时间，单位 s |
| 应用配置 | 原子更新参数并触发一次新的单点任务 |

姿态编号：

| 值 | 姿态 |
|---:|---|
| 1 | 攻击 |
| 2 | 防御 |
| 3 | 移动 |
| 4 | 强化攻击 |
| 5 | 强化防御 |
| 6 | 强化移动 |

## 对应运行时参数

| 参数 | 类型 | 默认值 |
|---|---|---|
| `runtime_goal_name` | string | `init` |
| `runtime_use_custom_pose` | bool | `false` |
| `runtime_goal_x` | double | `0.0` |
| `runtime_goal_y` | double | `0.0` |
| `runtime_goal_yaw` | double | `0.0` |
| `runtime_move_posture` | int | `3` |
| `runtime_wait_posture` | int | `1` |
| `runtime_reach_threshold` | double | `0.25` |
| `runtime_wait_time_threshold` | double | `5.0` |

当“使用临时坐标”关闭时，`runtime_goal_name` 必须存在于路径点配置。启用时，行为树使用 X、Y、Yaw 生成名为 `waypoint_gui_runtime` 的临时目标。

## 参数校验

`sentry_nav_bt` 会拒绝以下配置：

- 目标点名称为空；
- 坐标或角度不是有限数值；
- 姿态不在 1～6；
- 到点距离不大于 0；
- 等待时间小于 0；
- 选择命名点位但名称不存在。

被拒绝时，面板底部会显示行为树返回的原因。原子参数服务保证不会只应用其中一部分参数。

## 测试

模型层测试不依赖图形界面：

```bash
colcon test --packages-select sentry_nav_bt_gui
colcon test-result --verbose
```

测试覆盖路径点 JSON 加载和参数类型规范化。

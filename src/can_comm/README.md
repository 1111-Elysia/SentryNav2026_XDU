# can_comm

本包包含底盘 CAN 通信相关节点，以及一个基于 TF 的原地转向（`vyaw`）控制节点。

## vyaw_tf_yaw_controller_node

### 功能概述

`vyaw_tf_yaw_controller_node` 会周期性读取 TF 中 `map_frame -> base_frame` 的 yaw 角，并通过发布 `geometry_msgs/msg/Twist` 到 `cmd_vel_topic` 来控制底盘绕 z 轴旋转（仅使用 `angular.z`）。

- 当 yaw 误差进入容差范围时：发布一次 `angular.z = 0`，随后**锁存成功**，不再发布任何 `vyaw`。
- 若 TF 查询失败：会发布一次 `angular.z = 0`（防止盲转）。

> 注意：该节点发布的 Twist 只有 `angular.z` 被赋值，其它分量为默认 0，这会覆盖其它来源的速度指令（若系统里有多源 `cmd_vel`，建议配合 mux/仲裁）。

### 输入/输出

- 输入（TF）：`map_frame` 到 `base_frame` 的变换（默认 `map -> base_link`）
- 输出（Topic）：`cmd_vel_topic`（默认 `/cmd_vel`）
  - 发布 `geometry_msgs/msg/Twist`，其中 `angular.z` 表示期望角速度（rad/s）

### 目标 yaw 的确定方式

节点**不会从参数读取 target yaw**，而是在每次收到 `/yaw_controller=true` 触发后，基于参数给定的点 A 与当前 base_link 的 map 坐标点 B，构造直角三角形计算一次目标 yaw，并在本次控制过程中**缓存**该目标值。

#### 几何图示（ROS 标准 map：+x 向前、+y 向左）

按项目约定：A 在 map 的 x 正半轴（常用配置 `a_y = 0`），且 `a_x > b_x`，`b_y > 0`。

```
map-y (+y) ↑
           |
           |   B(bx,by) --------- C(ax,by)
           |                        |
           |                        |
           |                       A(ax,ay)
           +--------------------------------→ map-x (+x)

BC = |ax - bx|
AC = |by - ay|
```

- 取角：在 B 点取锐角 $\theta$，并要求 $\theta$ 为正值
- 定义：$\tan\theta = \dfrac{AC}{BC}$

因此

$$
\theta = \arctan\left(\frac{AC}{BC}\right) = \mathrm{atan2}(AC, BC)
$$

目标角（角度制）定义为：

$$
\text{target\_yaw\_deg} = 180^\circ - \theta
$$

实现上使用 $AC=|b_y-a_y|$、$BC=|a_x-b_x|$ 来“保留正值”。当 $BC\to 0$（A 与 B x 相同）时，按退化情况令 $\theta=90^\circ$。

### 控制律

- yaw 误差（弧度）：
  $$e = \mathrm{wrap}(\psi_{target} - \psi_{current})$$
- 指令角速度（带最小/最大绝对值限幅）：
  $$|\omega| = \mathrm{clamp}(k_p|e|,\ \text{min\_abs\_vyaw},\ \text{max\_abs\_vyaw})$$
  $$\omega = \mathrm{sign}(e)\cdot|\omega|$$

### 参数说明

- `cmd_vel_topic`（string，默认 `/cmd_vel`）：输出速度话题
- `map_frame`（string，默认 `map`）：TF 的源坐标系
- `base_frame`（string，默认 `base_link`）：TF 的目标坐标系

- `yaw_tolerance_deg`（double，默认 `3.0`）：到达判定容差（角度制）

- `a_x`（double，默认 `0.0`）：点 A 的 x（map 下）
- `a_y`（double，默认 `0.0`）：点 A 的 y（map 下）
- `a_yaw_deg`（double，默认 `0.0`）：当前版本计算中未使用，保留为兼容/扩展

- `min_abs_vyaw`（double，默认 `0.15`）：最小角速度绝对值（rad/s）
- `max_abs_vyaw`（double，默认 `1.0`）：最大角速度绝对值（rad/s）
- `k_p`（double，默认 `1.2`）：比例增益（输入为弧度误差）

- `control_frequency`（double，默认 `30.0`）：控制频率（Hz）

### “首次成功后不再发布”行为（锁存）

- 首次进入容差：发布一次 `angular.z=0`，将内部 `reached_once_` 置为 true，并停止本次控制（后续定时回调直接 return，不再发布）。
- 锁存后：回调直接 return，不再发布任何 `vyaw`（包括 0）。
- 析构时：仅在未成功锁存时才会发布一次 `angular.z=0`。

### 运行示例

1) 使用 A 点自动计算：

```bash
ros2 run can_comm vyaw_tf_yaw_controller_node --ros-args \
  -p a_x:=5.0 -p a_y:=0.0 \
  -p yaw_tolerance_deg:=3.0
```

3) 通过 launch 启动：见 `launch/can_comm.launch.py`。该 launch 会为该节点加载包内 `config/vyaw_tf_yaw_controller.yaml`（可直接在包内修改该配置）。

ros2 topic pub -1 /yaw_controller std_msgs/msg/Bool "{data: true}"
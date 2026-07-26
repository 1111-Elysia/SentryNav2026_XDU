<div align="center">
  <h1>⚡Super-LIO</h1>
  <h2>Super-LIO：一种鲁棒高效的激光雷达-惯性里程计系统，采用紧凑建图策略</h2>
  <p><strong>该工作已被 <i>IEEE Robotics and Automation Letters (RA-L 2026)</i> 接收。</strong></p>
  <br>

  [![Code](https://img.shields.io/badge/GitHub-181717?style=flat-square&logo=github&logoWidth=16)](https://github.com/Liansheng-Wang/Super-LIO.git) [![arXiv](https://img.shields.io/badge/arXiv-blue?logo=arxiv&color=%23B31B1B)](https://arxiv.org/abs/2509.05723) [![IEEE](https://img.shields.io/badge/RAL2026-004088.svg)](https://ieeexplore.ieee.org/document/11347459) [![Bilibili](https://img.shields.io/badge/Bilibili-00A1D6?style=flat-square&logo=bilibili&logoColor=white&logoWidth=16)](https://www.bilibili.com/video/BV11wBeBYEp6) [![YouTube](https://img.shields.io/badge/YouTube-FF0000?style=flat-square&logo=youtube&logoColor=white&logoWidth=16)](https://youtu.be/m9-hl8s5DDw)
</div>


<div align="center">
  <p>
    <a href="https://github.com/Liansheng-Wang/Super-LIO/tree/ros1" style="text-decoration: none;">
      <img src="https://img.shields.io/badge/🔄 切换 - ROS1 Noetic-3b82f6?style=for-the-badge&logo=ros&logoColor=white&logoWidth=22" alt="Switch to ROS1"
      onmouseover="this.src='https://img.shields.io/badge/🔄 切换 - ROS1 Noetic-1e40af?style=for-the-badge&logo=ros&logoColor=white&logoWidth=22'"
      onmouseout="this.src='https://img.shields.io/badge/🔄 切换 - ROS1 Noetic-3b82f6?style=for-the-badge&logo=ros&logoColor=white&logoWidth=22'"/>
    </a>&nbsp;&nbsp;
    <a href="https://github.com/Liansheng-Wang/Super-LIO/tree/ros2" style="text-decoration: none;">
      <img src="https://img.shields.io/badge/✅ 活跃 - ROS2 Humble/Iron/Jazzy-22c55e?style=for-the-badge&logo=ros&logoColor=white&logoWidth=22" alt="ROS2 Active"
      onmouseover="this.src='https://img.shields.io/badge/✅ 活跃 - ROS2 Humble/Iron/Jazzy-166534?style=for-the-badge&logo=ros&logoColor=white&logoWidth=22'"
      onmouseout="this.src='https://img.shields.io/badge/✅ 活跃 - ROS2 Humble/Iron/Jazzy-22c55e?style=for-the-badge&logo=ros&logoColor=white&logoWidth=22'"/>
    </a>&nbsp;&nbsp;
    <a href="#" style="text-decoration: none; cursor: default;">
      <img src="https://img.shields.io/badge/🖥️ 平台 - X86 + ARM64-8b5cf6?style=for-the-badge&logo=linux&logoColor=white&logoWidth=22" alt="X86 and ARM Support"
      onmouseover="this.src='https://img.shields.io/badge/🖥️ 平台 - X86_64 + ARM64-4f46e5?style=for-the-badge&logo=linux&logoColor=white&logoWidth=22'"
      onmouseout="this.src='https://img.shields.io/badge/🖥️ 平台 - X86_64 + ARM64-8b5cf6?style=for-the-badge&logo=linux&logoColor=white&logoWidth=22'"/>
    </a>
  </p>
</div>

## 概述

<p align="center">
  <img src="docs/system_overview.png" width="95%">
</p>

**核心特性：高效 · 鲁棒 · 跨平台兼容 · 同时支持 ROS1/ROS2**

Super-LIO 是一个鲁棒高效的激光雷达-惯性里程计（LIO）系统，专为实时大规模自主导航设计。它引入了一种紧凑且结构化的建图策略，实现了可预测的关联搜索和稳定的状态估计。该系统通过大量真实世界实验和与最先进方法的对比验证，表明 Super-LIO 不仅具备**出色的精度**，而且保持**更低的资源消耗**，并实现了近 **1.2–4 倍的实时处理速度提升**⚡。


**贡献者**：[Liansheng Wang](https://github.com/Liansheng-Wang), [Xinke Zhang](https://github.com/PSQzzzxk), [Chenhui Li](https://github.com/kermitLHH), [Dongjiao He](https://github.com/Joanna-HE), [Yihan pan](https://github.com/pyh3552), Jianjun Yi.


## 快速开始

**ROS1 用户**：请切换到 **ros1** 分支并参考 [ros1 分支](https://github.com/Liansheng-Wang/Super-LIO/tree/ros1) 的说明。

### 环境要求

Ubuntu 24(22).04 · C++20 · ROS Jazzy(Humble) · Eigen · PCL

### 依赖

glog · TBB

```bash
sudo apt install libgoogle-glog-dev libtbb-dev
```

### 编译与运行
```bash
git clone https://github.com/Liansheng-Wang/Super-LIO.git
cd Super-LIO
colcon build

source install/setup.bash



```

#### 🔁 重定位模式
Super-LIO 支持利用预建地图进行重定位，可在不重新建图的情况下从已保存的地图中恢复定位。
该模式适用于长期部署、重复任务或跟踪丢失后的恢复。

运行重定位前，请确保：
- 已有一个之前保存到磁盘的地图。

```bash
cd PATH_2_Super-LIO
source install/setup.bash
ros2 launch super_lio relocation.py
```


## 数据集
<p align="center">
  <img src="docs/datasets_compressed.png" width="95%">
</p>

Super-LIO 在多个真实世界数据集上进行了评估，涵盖室内、室外和大规模场景。

> **待办**：数据集下载链接和详细描述将在未来提供。


---

## 论文引用

如果你喜欢我们的工作，请引用我们并给一个 star 🌟 支持我们。
如果你觉得该库对你有帮助，我们诚挚建议引用[我们的论文](https://ieeexplore.ieee.org/document/11347459)：

```latex
@article{Wang2026SuperLIO,
  author  = {Liansheng Wang and Xinke Zhang and Chenhui Li and Dongjiao He and Yihan Pan and Jianjun Yi},
  title   = {Super-LIO: A Robust and Efficient LiDAR-Inertial Odometry System With a Compact Mapping Strategy},
  journal = {IEEE Robotics and Automation Letters},
  year    = {2026},
  doi     = {10.1109/LRA.2026.3653372}
}
```


## 更新日志

<details>
<summary>点击展开 <b>更新日志</b>（点击折叠）</summary>

<br>

- 2026-01-04
  - 分离 ROS 接口与算法。
  - 重构 SuperLIOReLoc 使其继承自 SuperLIO。
  - 代码风格与 ROS2 版本对齐。

- 2026-01-04
  - 主分支重命名为 ros1
  - 新增 ros2 分支

- 2026-01-04 21:51
  - 发布 ROS2 版本

</details>
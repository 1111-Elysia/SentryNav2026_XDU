# Odin_ROS_Driver 使用说明

Odin 传感器模块的 ROS2 驱动套件（Manifold Tech Ltd.）

Odin1 维基： https://manifoldtechltd.github.io/wiki/Odin1/Cover.html

## Odin_ROS_Driver

兼容性：

● ROS 2（LTS：推荐 Humble）

## 重要声明

本驱动包为点云 SLAM 等应用提供核心能力，并面向特定使用场景。该软件仅供具备二次开发能力的技术人员使用。最终用户需要根据自身场景进行针对性的优化与定制开发，以满足实际部署环境中的运行需求。

## 1. 版本

当前版本：v0.9.0

所需设备固件版本：v0.10.0

## 2. 环境准备

### 2.1 操作系统要求

● Ubuntu 22.04（ROS2 Humble）；

● Ubuntu 18.04：当前不支持；

● Ubuntu 24.04：非官方支持，可能需要额外修改。

### 2.2 依赖项

● OpenCV >= 4.5.0（推荐 4.5.5/4.8.0；请确保系统中只安装并使用一套 OpenCV）

● yaml-cpp

● thread

● OpenSSL

● Eigen3

### 2.3 依赖安装

#### 2.3.1 基础工具
```shell
sudo apt update
sudo apt-get install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
```

#### 2.3.2 yaml-cpp
```shell
sudo apt update
sudo apt install -y libyaml-cpp-dev
```

#### 2.3.3 libusb
```shell
sudo apt update
sudo apt install -y libusb-1.0-0-dev
```

#### 2.3.4 OpenCV
```shell
sudo apt update
sudo apt-get install libopencv-dev
```

#### 2.3.5 ROS 安装
ROS2 Humble 的安装请参考：
[ROS Humble 安装文档](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

## 3. 编译与运行

### 3.1 配置 Udev 规则
```shell
sudo vim /etc/udev/rules.d/99-odin-usb.rules
```
在 99-odin-usb.rules 中加入如下内容：
```shell
SUBSYSTEM=="usb", ATTR{idVendor}=="2207", ATTR{idProduct}=="0019", MODE="0666", GROUP="plugdev"
```
重新加载规则并重新插入设备：
```shell
sudo udevadm control --reload
sudo udevadm trigger
```

### 3.2 获取源码
```shell
git clone https://github.com/manifoldsdk/odin_ros_driver.git ros2_ws/src/odin_ros_driver
```
注意：
请将源码克隆到“[ros_workspace]/src/”目录下，否则可能出现编译错误。

### 3.3 编译

#### 3.3.1 ROS2（以 Humble 为例）

```shell
source /opt/ros/humble/setup.bash
colcon build --packages-select odin_ros_driver
```

### 3.4 运行

#### 3.4.1 ROS2（以 Humble 为例）

```shell
source [ros2_workspace]/install/setup.bash
ros2 launch odin_ros_driver [launch file]
```
● odin_ros_driver：包名；

● launch file：launch 文件名；

● ros2_workspace：用户的 ROS2 工作区路径；

ROS2 示例启动命令：
```shell
ros2 launch odin_ros_driver odin1_ros2.launch.py
```

### 3.5 工作模式

工作模式可通过 config/control_command.yaml 中的 `custom_map_mode` 参数配置。

#### 里程计模式（Odometry）

设置 `custom_map_mode = 0` 启用里程计模式。在该模式下，map 坐标系与 odom 坐标系保持同一位姿。

如果发现 odom 数据发生漂移，可在驱动源码目录执行脚本命令 "./set_param.sh algo_reset 1" 动态重置算法。

#### SLAM 模式

设置 `custom_map_mode = 1` 启用 SLAM 模式。该模式在里程计模式基础上增加**回环检测**与**地图保存**能力，从而提供完整 SLAM 系统。

启动驱动后，odin1 会自动建图并缓存地图数据。场景采集完成后，用户需要在驱动源码目录执行 `./set_param.sh save_map 1`，将程序启动以来收集到的地图数据保存到文件。地图保存路径由 config/control_command.yaml 中的 `mapping_result_dest_dir` 与 `mapping_result_file_name` 参数决定；若未指定，将使用默认路径与默认文件名。

首次保存后，你可以再次执行该命令来保存新的地图；每次保存都会生成一个新地图文件。（两次保存之间建议至少间隔 5 秒）

地图原点对应程序启动时 odom 坐标系的原点。

##### 重定位模式（Relocalization）

要启用重定位，将 `custom_map_mode = 2`，并在 config/control_command.yaml 中通过 `relocalization_map_abs_path` 指定预先构建好的地图文件绝对路径。

启动后，odin1 将基于当前视角与指定地图进行重定位。为获得更高成功率，建议在 SLAM 轨迹的原始位置与姿态附近启动：距离 1 米以内、角度 ±10° 以内。

重定位性能高度依赖环境。在特征明显的场景中，即使超过 1m/10° 范围也可能匹配成功；而在其他场景可能需要更严格条件。建议在目标环境中实际测试，以确定可用容差。

如果初次重定位失败，系统会暂时进入回退 SLAM 模式（该状态下不可保存地图）。此期间你可以自由移动 odin1，系统会在后台持续尝试重定位。匹配成功后，会发布 map 与 odom 之间的 TF。（小提示：初始化后轻轻摇晃/移动设备有时有助于提高重定位成功率。）

以下话题发布在 odom 坐标系下：`/odin1/cloud_slam, /odin1/odom, /odin1/highodom and /odin1/path`。如需在 map 坐标系下使用，请应用 odom→map 的 TF。

## 4. 文件结构与数据格式
### 4.1 文件结构
```shell
Odin_ROS_Driver/                // ROS2 驱动包
    3rdparty/                   // 第三方库
    src/
        host_sdk_sample.cpp     // 示例源码
        yaml_parser.cpp         // 读取 yaml 参数的源码
        rawCloudRender.cpp      // 点云渲染相关源码
        depth_image_ros2_node.cpp // depth_image_ros2_node
        pcd2depth_ros2.cpp      // pcd2depth_ros2 的源码
        pointcloud_depth_converter.cpp // pointcloud_depth_converter 的源码
        cloud_reprojection_ros.cpp // 云重投影节点源码（ROS2）
        cloud_reprojector.cpp   // 云重投影核心逻辑
    lib/
        liblydHostApi_amd.a     // AMD 平台静态库
        liblydHostApi_arm.a     // ARM 平台静态库
    include/
        host_sdk_sample.h       // 示例头文件
        lidar_api_type.h        // API 数据结构定义
        lidar_api.h             // API 函数声明
        yaml_parser.h           // 读取参数头文件
        rawCloudRender.h        // 点云渲染相关 API
        data_logger.h           // 保存数据的日志相关
        depth_image_ros2_node.hpp // depth_image_ros2_node
        pointcloud_depth_converter.hpp // pointcloud_depth_converter
        cloud_reprojection_ros_node.hpp // cloud_reprojection_ros_node（ROS2）
        cloud_reprojector.hpp   // 云重投影核心类
    config/
        control_command.yaml    // 驱动控制参数
        calib.yaml              // 设备标定参数（每台设备不同，设备连接驱动时会从设备读取）
    launch_ROS2/
        odin1_ros2.launch.py    // ROS2 launch 文件
    script/
        build_ros2.sh           // ROS2 安装/编译脚本
    recorddata/                 // 录制的数据，可导入 MindCloud(TM) 后处理
    log/                        // 日志目录
        Driver_{timestamp}/     // 每次启动驱动生成一份日志目录
            Conn_{timestamp}/   // 每次设备连接生成一份连接日志
                dev_status.csv  // 设备状态日志
    README.md                   // 使用说明
    CMakeLists.txt              // CMake 构建文件
    License                     // 许可证文件
```

### 4.2 Launch 文件
| Launch 文件名         | 说明 |
|-----------------------|------|
| odin1_ros2.launch.py  | ROS2 示例启动文件（Odin1 基础操作 Demo） |

### 4.3 ROS 话题
Odin ROS 驱动的内部参数定义在 config/control_command.yaml。下面是常用参数/话题说明：

| Topic                           | control_command.yaml | 详细说明 |
|--------------------------------|----------------------|----------|
| odin1/imu                      | sendimu              | IMU 话题 |
| odin1/image                    | sendrgb              | RGB 相机图像（由设备端 JPEG 解码得到，bgr8 格式） |
| odin1/image_undistort          | sendrgbundistort     | 去畸变 RGB 图像（基于设备提供的 calib.yaml 处理） |
| odin1/image/compressed         | sendrgbcompressed    | RGB 压缩图像（设备原始 JPEG 数据） |
| odin1/cloud_raw                | senddtof             | 原始点云（Raw_Cloud） |
| odin1/cloud_render             | sendcloudrender      | 渲染点云（由 raw 点云 + rgb 图像 + calib.yaml 计算） |
| odin1/cloud_slam               | sendcloudslam        | SLAM 点云 |
| odin1/odometry                 | sendodom             | 里程计（Odom） |
| odin1/odometry_high            | sendodom             | 高频里程计 |
| odin1/path                     | showpath             | 里程计轨迹（Path） |
| tf                             | sendodom             | TF 树 |
| odin1/depth_img_competetion    | senddepth            | 稠密深度图（Demo，计算资源消耗高；与 odin1/image_undistort 一一对应。建议直接订阅而不是 echo；该话题本身已是深度数据，无需再次转换） |
| odin1/depth_img_competetion_cloud | senddepth         | 稠密深度点云（Demo，计算资源消耗高） |
| odin1/reprojected_image        | sendreprojection     | 重投影图像（使用里程计将 cloud_slam 投影到相机图像上，主机侧处理） |

### 4.4 数据格式

1. 原始点云（cloud_raw）包含如下字段：
```
float32 x             // X 轴，单位：米
float32 y             // Y 轴，单位：米
float32 z             // Z 轴，单位：米
uint8  intensity      // 反射强度，范围 0–255
uint16 confidence     // 点置信度：典型场景下取值范围 0 到约 1300；值越大越可靠。推荐过滤阈值 30–35，需按场景调整。
float32 offset_time   // 相对基准时间戳的偏移量，单位：秒
```

若要在 PCL 中使用该自定义格式，请先定义点类型：
```cpp
/*** LS ***/
namespace ls_ros {
    struct EIGEN_ALIGN16 Point {
        float x;
        float y;
        float z;
        uint8_t intensity;
        uint16_t confidence;
        float offset_time;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}  // namespace ls_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(ls_ros::Point,
      (float, x, x)
      (float, y, y)
      (float, z, z)
      (uint8_t, intensity, intensity)
      (uint16_t, confidence, confidence)
      (float offset_time , offset_time)
)
```
随后可将 ROS 的 sensor_msgs::PointCloud2 转成 PCL 点云：
```
pcl::PointCloud<ls_ros::Point> ls_cloud;
pcl::fromROSMsg(*msg, ls_cloud);
```

2. SLAM 点云（cloud_slam）与直接渲染点云（cloud_render）包含如下字段：
```
float32 x             // X 轴，单位：米
float32 y             // Y 轴，单位：米
float32 z             // Z 轴，单位：米
float32 rgb           // RGB 值
```

### 4.5 其他功能参数说明

| control_command.yaml                         | 详细说明 |
|---------------------------------------------|----------|
| use_host_ros_time                             | 时间同步模式：0 - 使用 odin 内部系统时间作为数据时间戳（典型且推荐）；1 - 使用主机 ROS 接收时刻作为时间戳（一般不推荐）；2 - 将 odin1 时间对齐到主机时间（类似 NTP），时间戳为主机时间轴上的“主机接收时刻”。 |
| strict_usb3.0_check                           | 严格 USB3.0 检查：关闭时，即使连接不满足 USB3.0 也允许连接（仍强烈建议使用 USB3.0）。 |
| recorddata                                    | 以专有格式录制数据（可导入 MindCloud(TM) 后处理）。注意：会占用大量存储空间；测试数据显示 10 分钟约 9.5G。 |
| devstatuslog                                  | 设备状态记录：保存设备状态（SoC 温度、CPU 使用率、内存、dtof 传感器温度等）以及数据收发速率到 log 目录下的 dev_status.csv。每次启动驱动都会创建新文件。 |
| showcamerapose                                | 显示相机位姿与视场。 |
| custom_map_mode                               | 工作模式：0 - 里程计模式（map 与 odom 同位姿）；1 - 建图模式（含回环），支持保存地图；2 - 重定位模式，需要指定地图绝对路径；重定位成功后将输出 map 与 odom 的 TF 关系。 |
| custom_init_pos                               | 初始化位姿（当前未使用）。 |
| relocalization_map_abs_path                   | 地图文件绝对路径：用于重定位模式。 |
| mapping_result_dest_dir and mapping_result_file_name | 建图模式保存地图的路径与文件名：未指定时使用默认值。 |

## 5. 常见问题（FAQ）
### 5.1 重新启动 host SDK 触发 Segmentation fault
**报错信息**  
No device connected after 60 seconds

**解决方法**  
1. 重新给 Odin 模块上电（断电后再上电）

2. 重新初始化 Odin SDK（设备重启后再次执行 SDK）

### 5.2 编译时报库绑定失败

**报错信息**  
ld: cannot find -llydHostApi or symbol lookup errors

**处理方法**

1. 清理历史构建产物

ROS2
```shell
rm -rf build/ install/ log/
```
2. 重新运行安装/编译脚本

### 5.3 Docker GUI 透传失败

**报错信息**  
Unable to open X display or No protocol specified

**处理方法**
```shell
xhost + # 该命令用于启用 Docker 容器的图形界面透传
```

### 5.4 ROS 驱动提示 get version failed 并退出

**报错信息**
```shell
<ERROR><api.cpp:lidar_get_version:672>: get device version fail.
get version failed.
```

**处理方法**

设备固件版本过低，请升级到最新版本。

### 5.5 RViz 长时间无响应

**现象/报错信息**
Rviz does not respond, and after a while the terminal prints Device disconnected, waiting for reconnection...

**处理方法**

请重新给 Odin 模块上电。

### 5.6 设备无响应

**报错信息**
Missed ok response from device,probably wrong interaction procedure.

**处理方法**

请参考 5.1 的处理步骤。

### 5.7 设备没有外部标定文件

**报错信息**
ERROR：Missing camera node 'cam_0'

**处理方法**

请重新插拔 USB。

### 5.8 数据流启动后立刻断连

**报错信息**
```shell
Device ready and streams activated
Device detaching...
Wating for device reconnection...
Device disconnected, waiting for reconnection...
```

**原因**

该问题多发生在 ROS2 环境且网络环境复杂（如办公室 WiFi + 有线网并存）的场景。ROS2 默认使用广播发现，复杂网络可能导致 ROS2 发布阻塞，从而引发设备断连。

**处理方法**

若不需要跨设备通信，可将 ROS2 限制为仅本机通信：
```shell
export ROS_LOCALHOST_ONLY=1
```

若需要跨设备通信，建议尽可能简化网络环境；推荐搭建仅包含必要设备的小型局域网。

### 5.9 数据流启动后驱动进程立刻退出

**报错信息**
```shell
Device ready and streams activated
[host_sdk_sample-2] process has died ......
```

**测试方法**

在 control_command.yaml 中将 sendrgb = 0（禁用 odin1/image）后再试。如果此时驱动可以正常运行，通常意味着系统中存在多套 OpenCV 版本导致问题。

**处理方法**

清理/卸载多余 OpenCV，确保系统中仅保留一套完整 OpenCV 后，重新编译驱动再试。

### 5.10 RViz 出现 "TF_OLD_DATA ignoring data" 警告

**报错信息**
```shell
[rviz2-3] Warning: TF_OLD_DATA ignoring data from the past for frame odin1_base_link at time 20.547632 according to authority Authority undetectable
[rviz2-3] Possible reasons are listed at http://wiki.ros.org/tf/Errors%20explained
[rviz2-3]          at line 294 in ./src/buffer_core.cpp
```

**原因**

这是 ROS/tf 与 rviz 的提示机制：当时间戳冲突导致部分 TF 数据被忽略时，会输出该警告。常见场景是：保持驱动运行的情况下对设备进行断电重启，导致设备内部时间被重置，新数据时间戳与上一次运行期间 rviz 缓存的旧数据冲突。

**处理方法**

rviz 界面底部有一个 reset 按钮，点击后可重置 rviz 的内部状态并停止该警告。

### 5.11 驱动打印 "unknown cmd code: xx" 错误

**报错信息**
```shell
<ERROR><api.cpp:cmd_data_deal:418>: unknow command code 21.
```

**原因**

驱动版本与设备固件版本不匹配，导致驱动无法解析新固件新增的数据。

**处理方法**

请确保使用最新版本的驱动与对应的设备固件。

## 6. 联系方式

如需支持，请联系：support@manifoldtech.cn

为便于诊断问题，请向 FAE 提供以下信息：

1. 当前固件版本
```shell
[device_version_capture]: ros_driver_version: [Version Number]
```
2. 当前使用的电源适配器与转换线照片

3. 问题是偶发还是必现

4. 问题场景的图片

5. 是否已尝试第 5 节（FAQ）中的排查方法，结果如何

6. 期望的问题解决时间窗口

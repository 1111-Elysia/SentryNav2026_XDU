"""
测试 Adaptive MPPI 控制器 — 标准导航，无追踪功能。

用法:
  ros2 launch elastic_tracker_sim test_adaptive_mppi.launch.py

操作:
  1. 在 RViz 中使用 "2D Goal Pose" 工具设置目标点
  2. 机器人使用 pri_adaptive_mppi 控制器导航到目标
  3. 观察 /FollowPath/adaptive_line_visualization 话题中的直线和膨胀层
  4. 通过日志查看模式切换: ros2 run rqt_console rqt_console
"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("elastic_tracker_sim")

    # ── Process XACRO ────────────────────────────
    pursuer_xacro = os.path.join(pkg, "urdf", "omni_robot.urdf.xacro")
    pursuer_urdf = xacro.process_file(pursuer_xacro).toxml()

    pur_path = "/tmp/pursuer_test.urdf"
    with open(pur_path, "w") as f:
        f.write(pursuer_urdf)

    # ── Map & Nav2 params ────────────────────────
    map_yaml = os.path.join(pkg, "map.yaml")
    nav2_params = os.path.join(pkg, "config", "test_adaptive_mppi_params.yaml")

    return LaunchDescription(
        [
            # ======================================================
            #  t=0 : Gazebo
            # ======================================================
            ExecuteProcess(
                cmd=[
                    "gazebo", "--verbose",
                    os.path.join(pkg, "worlds", "empty.world"),
                    "-s", "libgazebo_ros_init.so",
                    "-s", "libgazebo_ros_factory.so",
                ],
                output="screen",
            ),
            # ======================================================
            #  t=2 : 机器人 + TF
            # ======================================================
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package="gazebo_ros",
                        executable="spawn_entity.py",
                        arguments=[
                            "-entity", "pursuer", "-file", pur_path,
                            "-x", "0", "-y", "0", "-z", "0.15",
                        ],
                    ),
                    Node(
                        package="robot_state_publisher",
                        executable="robot_state_publisher",
                        name="pur_state",
                        parameters=[{"robot_description": pursuer_urdf, "use_sim_time": True}],
                    ),
                    Node(
                        package="tf2_ros",
                        executable="static_transform_publisher",
                        arguments=[
                            "--x", "0", "--y", "0", "--z", "0",
                            "--roll", "0", "--pitch", "0", "--yaw", "0",
                            "--frame-id", "map", "--child-frame-id", "odom",
                        ],
                        parameters=[{"use_sim_time": True}],
                    ),
                ],
            ),
            # ======================================================
            #  t=3 : RViz
            # ======================================================
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        arguments=["-d", os.path.join(pkg, "rviz", "sim.rviz")],
                    ),
                ],
            ),
            # ======================================================
            #  t=4 : Map Server
            # ======================================================
            TimerAction(
                period=4.0,
                actions=[
                    Node(
                        package="nav2_map_server",
                        executable="map_server",
                        name="map_server",
                        parameters=[{"yaml_filename": map_yaml, "use_sim_time": True}],
                    ),
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_map",
                        parameters=[
                            {"use_sim_time": True, "node_names": ["map_server"], "autostart": True}
                        ],
                    ),
                ],
            ),
            # ======================================================
            #  t=5 : Nav2 (标准导航 + pri_adaptive_mppi)
            # ======================================================
            TimerAction(
                period=5.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(
                                get_package_share_directory("nav2_bringup"),
                                "launch",
                                "navigation_launch.py",
                            )
                        ),
                        launch_arguments={
                            "params_file": nav2_params,
                            "use_sim_time": "true",
                        }.items(),
                    ),
                ],
            ),
            # ======================================================
            #  t=6 : 启动提示
            # ======================================================
            TimerAction(
                period=6.0,
                actions=[
                    LogInfo(
                        msg="\n\n══════════════════════════════════════════\n"
                        "  Adaptive MPPI 测试环境就绪\n"
                        "  控制器: pri_adaptive_mppi::PriAdaptiveMppi\n"
                        "  规划器: SmacPlanner2D\n"
                        "  操作: 在 RViz 中使用 2D Goal Pose 设置目标\n"
                        "  观察: /FollowPath/adaptive_line_visualization\n"
                        "══════════════════════════════════════════\n"
                    ),
                ],
            ),
        ]
    )

"""
Elastic-Tracker Simulation — one-click launch.

Starts Gazebo (pursuer + target), Map Server, RViz, Nav2 (MPPI + ElasticTracker),
teleop for target, and auto-relay target position → /detected_target_pose.

Usage:
  ros2 launch elastic_tracker_sim sim.launch.py

After launch:
  - Use RViz "2D Goal Pose" to set initial navigation goal
  - Use teleop terminal to drive the RED target robot
  - The pursuer (BLUE) will autonomously track it
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
    target_xacro = os.path.join(pkg, "urdf", "target.urdf.xacro")
    pursuer_urdf = xacro.process_file(pursuer_xacro).toxml()
    target_urdf = xacro.process_file(target_xacro).toxml()

    pur_path = "/tmp/pursuer.urdf"
    tgt_path = "/tmp/target.urdf"
    with open(pur_path, "w") as f: f.write(pursuer_urdf)
    with open(tgt_path, "w") as f: f.write(target_urdf)

    # ── Map & Nav2 params ────────────────────────
    map_yaml = os.path.join(pkg, "map.yaml")
    nav2_params = os.path.join(pkg, "config", "sim_nav2_params.yaml")

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
            #  t=2 : Pursuer + TF
            # ======================================================
            TimerAction(
                period=2.0, actions=[
                    Node(package="gazebo_ros", executable="spawn_entity.py",
                         arguments=["-entity","pursuer","-file",pur_path,
                                    "-x","0","-y","0","-z","0.15"]),
                    Node(package="robot_state_publisher", executable="robot_state_publisher",
                         name="pur_state",
                         parameters=[{"robot_description": pursuer_urdf, "use_sim_time": True}]),
                    Node(package="tf2_ros", executable="static_transform_publisher",
                         arguments=["--x","0","--y","0","--z","0","--roll","0","--pitch","0","--yaw","0",
                                    "--frame-id","map","--child-frame-id","odom"],
                         parameters=[{"use_sim_time": True}]),
                ],
            ),
            # ======================================================
            #  t=2.5 : Target + TF bridge
            # ======================================================
            TimerAction(
                period=2.5, actions=[
                    Node(package="gazebo_ros", executable="spawn_entity.py",
                         arguments=["-entity","target","-file",tgt_path,
                                    "-x","3","-y","2","-z","0.15",
                                    "-robot_namespace","target"]),
                    Node(package="robot_state_publisher", executable="robot_state_publisher",
                         name="tgt_state", namespace="target",
                         parameters=[{"robot_description": target_urdf, "use_sim_time": True}]),
                    Node(package="tf2_ros", executable="static_transform_publisher",
                         arguments=["--x","0","--y","0","--z","0","--roll","0","--pitch","0","--yaw","0",
                                    "--frame-id","odom","--child-frame-id","target/odom"],
                         parameters=[{"use_sim_time": True}]),
                ],
            ),
            # ======================================================
            #  t=3 : RViz
            # ======================================================
            TimerAction(
                period=3.0, actions=[
                    Node(package="rviz2", executable="rviz2",
                         arguments=["-d", os.path.join(pkg, "rviz", "sim.rviz")]),
                ],
            ),
            # ======================================================
            #  t=4 : Map Server
            # ======================================================
            TimerAction(
                period=4.0, actions=[
                    Node(package="nav2_map_server", executable="map_server", name="map_server",
                         parameters=[{"yaml_filename": map_yaml, "use_sim_time": True}]),
                    Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
                         name="lifecycle_manager_map",
                         parameters=[{"use_sim_time": True, "node_names": ["map_server"], "autostart": True}]),
                ],
            ),
            # ======================================================
            #  t=5 : Nav2
            # ======================================================
            TimerAction(
                period=5.0, actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(get_package_share_directory("nav2_bringup"),
                                         "launch", "navigation_launch.py")),
                        launch_arguments={"params_file": nav2_params, "use_sim_time": "true"}.items(),
                    ),
                ],
            ),
            # ======================================================
            #  t=6 : Target relay + Teleop
            # ======================================================
            TimerAction(
                period=6.0, actions=[
                    Node(package="elastic_tracker_sim", executable="target_relay.py",
                         name="target_relay", output="screen",
                         parameters=[{"use_sim_time": True}]),
                    ExecuteProcess(
                        cmd=["gnome-terminal", "--", "ros2", "run", "teleop_twist_keyboard",
                             "teleop_twist_keyboard", "--ros-args", "-r", "/cmd_vel:=/target/cmd_vel"],
                        output="screen",
                    ),
                    LogInfo(
                        msg="\n\n══════════════════════════════════════════\n"
                        "  Simulation READY\n"
                        "  Planner:  ElasticTracker (default)\n"
                        "  Target:   RED robot (teleop in terminal)\n"
                        "  Pursuer:  BLUE robot (auto-tracking)\n"
                        "══════════════════════════════════════════\n"
                    ),
                ],
            ),
        ]
    )

"""CAN 调试 launch — 启动 GUI 面板 + can_send + can_receive"""
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    can_comm_share = get_package_share_directory("bringup")
    config_file = os.path.join(can_comm_share, "config", "can_params.yaml")

    return LaunchDescription([
        # 调试 GUI
        Node(
            package="can_comm",
            executable="can_debug_gui",
            name="can_debug_gui",
            output="screen",
        ),
        # CAN 发送 
        Node(
            package="can_comm",
            executable="can_comm_node",
            name="can_send",
            output="screen",
            parameters=[config_file],
        ),
        # CAN 接收
        Node(
            package="can_comm",
            executable="can_receive_node",
            name="can_receive",
            output="screen",
            parameters=[config_file],
        ),
        # Node(
        #     package='can_comm',
        #     executable='target_frame_node',
        #     name='target_frame',
        #     output='screen',
        #     parameters=[config_file],
        # ),
        # Node(
        #     package='can_comm',
        #     executable='vyaw_tf_yaw_controller_node',
        #     name='vyaw_tf_yaw_controller',
        #     output='screen',
        #     parameters=[config_file],
        # ),
    ])

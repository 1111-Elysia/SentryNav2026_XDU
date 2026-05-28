from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        # GUI 模拟器节点 (Python)
        Node(
            package='ground_pos_relay',
            executable='ground_pos_simulator.py',
            name='ground_pos_simulator',
            output='screen',
        ),
        # 模拟位置中继节点 (C++), 默认 robot_id=7
        Node(
            package='ground_pos_relay',
            executable='ground_pos_relay_sim_node',
            name='ground_pos_relay_sim_node',
            output='screen',
            parameters=[{'robot_id': 7}],
        ),
    ])

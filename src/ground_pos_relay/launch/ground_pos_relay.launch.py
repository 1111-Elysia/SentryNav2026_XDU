from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ground_pos_relay',
            executable='ground_pos_relay_node',
            name='ground_pos_relay_node',
            output='screen',
        ),
    ])

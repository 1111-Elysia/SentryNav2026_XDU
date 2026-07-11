from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('ground_pos_relay')
    config_file = os.path.join(pkg_share, 'config', 'ground_pos_relay.yaml')

    return LaunchDescription([
        Node(
            package='ground_pos_relay',
            executable='teammate_frame_converter_node',
            name='teammate_frame_converter_node',
            output='screen',
            parameters=[config_file],
        ),
    ])

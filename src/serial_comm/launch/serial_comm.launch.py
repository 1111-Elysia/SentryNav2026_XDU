from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('serial_comm')
    
    return LaunchDescription([
        Node(
            package='serial_comm',
            executable='serial_comm_node',
            name='serial_comm_node',
            output='screen',
            parameters=[os.path.join(pkg_share, 'config', 'serial_params.yaml')]
        )
    ])

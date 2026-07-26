import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('d435_process_pcl')
    
    config = os.path.join(
        pkg_dir,
        'config',
        'params.yaml'
    )

    depth_to_pcl_node = Node(
        package='d435_process_pcl',
        executable='depth_to_pcl_node',
        name='depth_to_pcl_node',
        output='screen',
        parameters=[config]
    )

    return LaunchDescription([
        depth_to_pcl_node
    ])

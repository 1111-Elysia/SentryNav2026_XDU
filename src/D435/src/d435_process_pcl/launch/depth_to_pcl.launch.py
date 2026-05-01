import os
import yaml
import math
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
    
    with open(config, 'r') as f:
        yaml_data = yaml.safe_load(f)
        
    params = yaml_data['/**']['ros__parameters']
    tf_x = str(params.get('tf_x', 0.0))
    tf_y = str(params.get('tf_y', 0.0))
    tf_z = str(params.get('tf_z', 0.0))
    
    # yaw, pitch, roll
    tf_yaw = str(math.radians(params.get('tf_yaw_deg', 0.0)))
    tf_pitch = str(math.radians(params.get('tf_pitch_deg', 0.0)))
    tf_roll = str(math.radians(params.get('tf_roll_deg', 0.0)))
    
    rviz_config = os.path.join(
        pkg_dir,
        'rviz',
        'd435.rviz'
    )

    depth_to_pcl_node = Node(
        package='d435_process_pcl',
        executable='depth_to_pcl_node',
        name='depth_to_pcl_node',
        output='screen',
        parameters=[config]
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        depth_to_pcl_node,
        rviz_node
    ])

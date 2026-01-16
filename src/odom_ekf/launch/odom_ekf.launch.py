from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包的共享目录路径，比硬编码路径更健壮
    share_dir = get_package_share_directory('odom_ekf')
    
    # 配置文件路径
    odom_list_params = os.path.join(share_dir, 'config', 'odom_list.yaml')
    # [修正] 文件名应为 ekf_config.yaml 而不是 ekf.yaml
    ekf_params = os.path.join(share_dir, 'config', 'ekf_config.yaml') 

    return LaunchDescription([
        Node(
            package='odom_ekf',
            executable='odom_preprocessor',
            name='odom_preprocessor',
            output='screen',
            parameters=[odom_list_params]
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params],
            # 将 EKF 默认的输出话题重映射为 /odom
            # remappings=[('odometry/filtered', '/odom')]s
        )
    ])
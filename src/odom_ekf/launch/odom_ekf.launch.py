from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share_dir = get_package_share_directory('odom_ekf')
    odom_list_params = os.path.join(share_dir, 'config', 'odom_list.yaml')
    ekf_params = os.path.join(share_dir, 'config', 'ekf_config.yaml') 

    return LaunchDescription([
        # [新增] 假如你的 IMU frame 是 livox_frame，这里发布一个到 livox_frame_two 的静态变换
        # 请根据上面 echo 出来的实际 frame_id 修改 args 的最后一个参数
        Node(
             package='tf2_ros',
             executable='static_transform_publisher',
             arguments = ['0', '0', '0', '0', '0', '0', 'livox_frame_two', 'livox_frame']
        ),

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
            # remappings=[('odometry/filtered', '/odom')]
        )
    ])
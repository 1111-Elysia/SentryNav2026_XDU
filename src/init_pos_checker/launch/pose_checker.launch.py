from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_pcd_path',
            default_value='',
            description='地图 PCD 路径（文件或目录）'
        ),
        DeclareLaunchArgument(
            'cloud_topic',
            default_value='/lio/cloud_world',
            description='定位系统输出的 map 坐标系下的点云话题'
        ),

        Node(
            package='init_pos_checker',
            executable='pose_checker_node',
            name='init_pos_checker',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'map_pcd_path':      LaunchConfiguration('map_pcd_path'),
                'cloud_topic':       LaunchConfiguration('cloud_topic'),
                'accumulate_frames': 5,
                'voxel_size':        0.3,
                'icp_max_iter':      50,
                'pos_threshold':     0.5,
                'rot_threshold':     5.0,
            }],
        ),
    ])
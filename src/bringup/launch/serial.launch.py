from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包 share 目录
    pkg_share = get_package_share_directory('serial_comm')

    # 绝对路径加载参数文件
    config_file = os.path.join(pkg_share, 'config', 'serial_params.yaml')
    topics_file = os.path.join(pkg_share, 'config', 'topic_names.yaml')  # 新增

    # 定义两个节点
    serial_node = Node(
        package='serial_comm',
        executable='serial_comm_node',
        name='serial_comm_node',
        output='screen',
        parameters=[config_file, topics_file]  # 同时加载两个参数文件
    )

    serial_receive_node = Node(
        package='serial_comm',
        executable='serial_receive_node',
        name='serial_receive_node',
        output='screen',
        parameters=[config_file, topics_file]  # 同样加载
    )

    # 返回 LaunchDescription
    return LaunchDescription([
        serial_node,
        serial_receive_node
    ])

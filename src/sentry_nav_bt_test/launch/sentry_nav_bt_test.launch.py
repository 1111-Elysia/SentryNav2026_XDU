import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 获取包的安装路径 (install/share/sentry_nav_bt_test)
    pkg_dir = get_package_share_directory('sentry_nav_bt_test')
    
    # 2. 定义文件路径
    # 行为树 XML
    bt_xml_path = os.path.join(pkg_dir, 'config', 'demo.xml')
    
    # 红方路径点 JSON
    waypoints_red_path = os.path.join(pkg_dir, 'config', 'waypoints_red.json')
    # 蓝方路径点 JSON
    waypoints_blue_path = os.path.join(pkg_dir, 'config', 'waypoints_blue.json')

    # 3. 检查文件是否存在 (可选，为了报错更清晰)
    for path_check in [bt_xml_path, waypoints_red_path, waypoints_blue_path]:
        if not os.path.isfile(path_check):
            raise FileNotFoundError(f"配置文件未找到: {path_check}")

    # 4. 声明启动参数 (允许命令行覆盖)
    bt_xml_arg = DeclareLaunchArgument(
        'bt_xml_filename',
        default_value=bt_xml_path,
        description='行为树XML文件的完整路径'
    )
    
    red_wp_arg = DeclareLaunchArgument(
        'waypoints_red_file',
        default_value=waypoints_red_path,
        description='红方路径点文件路径'
    )

    blue_wp_arg = DeclareLaunchArgument(
        'waypoints_blue_file',
        default_value=waypoints_blue_path,
        description='蓝方路径点文件路径'
    )
    
    bridge_node = Node(
        package='sentry_nav_bt_test',
        executable='compatibility_bridge.py',
        name='referee_bridge',
        output='screen'
    )

    # 5. 创建节点
    sentry_nav_bt_test = Node(
        package='sentry_nav_bt_test',
        executable='navigate_bt_node', # 你的可执行文件名
        name='sentry_nav_bt_test',
        output='screen',
        # 将路径作为参数传递给 C++ 节点
        parameters=[{
            'bt_xml_filename': LaunchConfiguration('bt_xml_filename'),
            'waypoints_red_file': LaunchConfiguration('waypoints_red_file'),
            'waypoints_blue_file': LaunchConfiguration('waypoints_blue_file'),
        }]
    )
    
    return LaunchDescription([
        bt_xml_arg,
        red_wp_arg,
        blue_wp_arg,
        sentry_nav_bt_test,
        bridge_node
    ])
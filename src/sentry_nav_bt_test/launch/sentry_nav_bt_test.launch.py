import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # 声明启动参数
    use_old_protocol_arg = DeclareLaunchArgument(
        'use_old_protocol',
        default_value='false',
        description='是否使用旧协议 (rm2)'
    )

    team_color_arg = DeclareLaunchArgument(
        'team_color',
        default_value='red',
        description='阵营颜色: red / blue'
    )

    # 获取包的安装路径 
    pkg_dir = get_package_share_directory('sentry_nav_bt_test')
    
    # 行为树XML文件路径
    # bt_xml_path = os.path.join(pkg_dir, 'config', 'ul_3.21.xml')
    bt_xml_path = os.path.join(pkg_dir, 'config', 'ul.xml')
    
    # 红方路径点 JSON
    waypoints_red_path = os.path.join(pkg_dir, 'config', 'waypoints_red.json')
    # 蓝方路径点 JSON
    waypoints_blue_path = os.path.join(pkg_dir, 'config', 'waypoints_blue.json')

    # 检查配置文件是否存在 
    for path_check in [bt_xml_path, waypoints_red_path, waypoints_blue_path]:
        if not os.path.isfile(path_check):
            raise FileNotFoundError(f"配置文件未找到: {path_check}")

    # 声明启动参数
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

    bt_message_log_arg = DeclareLaunchArgument(
        'bt_message_log_file',
        default_value='/tmp/sentry_nav_bt_messages.log',
        description='行为树 PrintNode 日志目录/基准文件路径，实际会按日期落盘，置空可关闭'
    )
    
    bridge_node = Node(
        package='sentry_nav_bt_test',
        executable='compatibility_bridge.py',
        name='referee_bridge',
        output='screen',
        parameters=[{
            'team_color': LaunchConfiguration('team_color')
        }],
        condition=IfCondition(LaunchConfiguration('use_old_protocol'))
    )

    # 创建节点
    sentry_nav_bt_test = Node(
        package='sentry_nav_bt_test',
        executable='navigate_bt_node',
        name='sentry_nav_bt_test',
        output='screen',
        parameters=[{
            'bt_xml_filename': LaunchConfiguration('bt_xml_filename'),
            'waypoints_red_file': LaunchConfiguration('waypoints_red_file'),
            'waypoints_blue_file': LaunchConfiguration('waypoints_blue_file'),
            'bt_message_log_file': LaunchConfiguration('bt_message_log_file'),
            'use_sim_time': False,
        }]
    )
    
    return LaunchDescription([
        bt_xml_arg,
        red_wp_arg,
        blue_wp_arg,
        bt_message_log_arg,
        use_old_protocol_arg,
        team_color_arg,
        sentry_nav_bt_test,
        bridge_node
    ])

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包的安装路径 
    pkg_dir = get_package_share_directory('sentry_nav_bt_test')
    
    # 行为树XML文件路径
    # bt_xml_path = os.path.join(pkg_dir, 'config', 'ul_3.21.xml')
    # bt_xml_path = os.path.join(pkg_dir, 'config', 'uc.xml')
    bt_xml_path = os.path.join(pkg_dir, 'config', 'bt', '5.3tree.xml')
    bt_subtree_dir = os.path.join(pkg_dir, 'config', 'bt', 'modules')

    # 路径点 JSON
    waypoints_path = os.path.join(pkg_dir, 'config', 'waypoints.json')

    # 检查配置文件是否存在 
    for path_check in [bt_xml_path, bt_subtree_dir, waypoints_path]:
        if not os.path.isfile(path_check):
            if not os.path.isdir(path_check):
                raise FileNotFoundError(f"配置文件未找到: {path_check}")

    # 声明启动参数
    bt_xml_arg = DeclareLaunchArgument(
        'bt_xml_filename',
        default_value=bt_xml_path,
        description='行为树XML文件的完整路径'
    )

    bt_main_tree_id_arg = DeclareLaunchArgument(
        'bt_main_tree_id',
        default_value='MainTree',
        description='主行为树 ID'
    )
    
    waypoints_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value=waypoints_path,
        description='路径点文件路径'
    )

    bt_message_log_arg = DeclareLaunchArgument(
        'bt_message_log_file',
        default_value='/tmp/sentry_nav_bt_messages.log',
        description='行为树 PrintNode 日志目录/基准文件路径；固定路径会累计所有运行记录，同时也会为每次启动生成唯一日志文件，置空可关闭'
    )

    bt_subtree_dir_arg = DeclareLaunchArgument(
        'bt_subtree_dir',
        default_value=bt_subtree_dir,
        description='行为树子树 XML 目录'
    )

    validate_bt_only_arg = DeclareLaunchArgument(
        'validate_bt_only',
        default_value='false',
        description='只加载校验行为树 XML，不等待 Nav2 action server，不执行 tick'
    )
    
    # 创建节点
    sentry_nav_bt_test = Node(
        package='sentry_nav_bt_test',
        executable='navigate_bt_node',
        name='sentry_nav_bt_test',
        output='screen',
        parameters=[{
            'bt_xml_filename': LaunchConfiguration('bt_xml_filename'),
            'bt_main_tree_id': LaunchConfiguration('bt_main_tree_id'),
            'waypoints_file': LaunchConfiguration('waypoints_file'),
            'bt_message_log_file': LaunchConfiguration('bt_message_log_file'),
            'bt_subtree_dir': LaunchConfiguration('bt_subtree_dir'),
            'validate_bt_only': LaunchConfiguration('validate_bt_only'),
            'use_sim_time': False,
        }]
    )
    
    return LaunchDescription([
        bt_xml_arg,
        bt_main_tree_id_arg,
        waypoints_arg,
        bt_message_log_arg,
        bt_subtree_dir_arg,
        validate_bt_only_arg,
        sentry_nav_bt_test
    ])

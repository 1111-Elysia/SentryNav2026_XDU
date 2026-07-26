import os
from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessIO, OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return value.strip().lower() in {'1', 'true', 'yes', 'on'}


class _TerminalLogWriter:
    def __init__(self, log_file):
        self._stream = open(log_file, 'ab', buffering=0)
        self._buffers = {1: b'', 2: b''}
        self._closed = False

    def write_start(self, event, _context):
        self._write_line(
            f"[INFO] [{event.process_name}]: process started with pid [{event.pid}]".encode()
        )

    def write_io(self, event):
        fd = 1 if event.from_stdout else 2
        data = self._buffers[fd] + event.text
        lines = data.splitlines(keepends=True)
        self._buffers[fd] = b''
        if lines and not lines[-1].endswith((b'\n', b'\r')):
            self._buffers[fd] = lines.pop()
        for line in lines:
            self._write_prefixed(event.process_name, line)

    def write_exit(self, event, _context):
        for fd in (1, 2):
            if self._buffers[fd]:
                self._write_prefixed(event.process_name, self._buffers[fd] + b'\n')
                self._buffers[fd] = b''
        self._write_line(
            f"[INFO] [{event.process_name}]: process exited with code [{event.returncode}]".encode()
        )
        self.close()

    def _write_prefixed(self, process_name, line):
        self._stream.write(f"[{process_name}] ".encode() + line)

    def _write_line(self, line):
        self._stream.write(line + b'\n')

    def close(self):
        if not self._closed:
            self._stream.close()
            self._closed = True


def generate_launch_description():
    # 获取包的安装路径 
    pkg_dir = get_package_share_directory('sentry_nav_bt_test')
    source_package_dir = Path(__file__).resolve().parents[1]
    
    # 行为树XML文件路径
    # bt_xml_path = os.path.join(pkg_dir, 'config', 'bt', 'uc_fortress.xml')
    # bt_xml_path = os.path.join(pkg_dir, 'config', 'bt', 'uc_myhero.xml')
    # bt_xml_path = os.path.join(pkg_dir, 'config', 'bt', 'uc_patrol.xml')  
    # bt_xml_path = os.path.join(pkg_dir, 'config', 'bt', 'uc_chase.xml')
    bt_xml_path = os.path.join(pkg_dir, 'config', 'bt', 'uc_adaptive_training.xml')
    # bt_xml_path = os.path.join(pkg_dir, 'config', 'bt', 'train.xml')
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
        default_value='',
        description='兼容用 PrintNode 专用日志路径；默认关闭，避免与完整终端日志重复'
    )

    save_terminal_log_arg = DeclareLaunchArgument(
        'save_terminal_log',
        default_value='true',
        description='是否将 navigate_bt_node 的完整终端输出实时保存到文件'
    )

    terminal_log_dir_arg = DeclareLaunchArgument(
        'terminal_log_dir',
        default_value=str(source_package_dir / 'logs'),
        description='完整终端日志根目录；每次启动会创建独立子目录'
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
    
    def launch_setup(context):
        save_terminal_log = _as_bool(LaunchConfiguration('save_terminal_log').perform(context))
        actions = []

        sentry_nav_bt_test = Node(
            package='sentry_nav_bt_test',
            executable='navigate_bt_node',
            name='sentry_nav_bt_test',
            output='screen',
            emulate_tty=True,
            ros_arguments=['--disable-external-lib-logs'],
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

        if save_terminal_log:
            terminal_log_root = Path(
                LaunchConfiguration('terminal_log_dir').perform(context)
            ).expanduser()
            if not terminal_log_root.is_absolute():
                terminal_log_root = source_package_dir / terminal_log_root
            session_name = datetime.now().strftime('%Y-%m-%d_%H-%M-%S') + f'_pid{os.getpid()}'
            session_dir = terminal_log_root / session_name
            session_dir.mkdir(parents=True, exist_ok=False)
            terminal_log_file = session_dir / 'launch.log'
            writer = _TerminalLogWriter(terminal_log_file)
            print(f'[sentry_nav_bt_test.launch] 完整终端日志: {terminal_log_file}')

            actions.extend([
                RegisterEventHandler(
                    OnProcessStart(
                        target_action=sentry_nav_bt_test,
                        on_start=writer.write_start,
                    )
                ),
                RegisterEventHandler(
                    OnProcessIO(
                        target_action=sentry_nav_bt_test,
                        on_stdout=writer.write_io,
                        on_stderr=writer.write_io,
                    )
                ),
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=sentry_nav_bt_test,
                        on_exit=writer.write_exit,
                    )
                ),
            ])

        actions.append(sentry_nav_bt_test)
        return actions
    
    return LaunchDescription([
        bt_xml_arg,
        bt_main_tree_id_arg,
        waypoints_arg,
        bt_message_log_arg,
        save_terminal_log_arg,
        terminal_log_dir_arg,
        bt_subtree_dir_arg,
        validate_bt_only_arg,
        OpaqueFunction(function=launch_setup),
    ])

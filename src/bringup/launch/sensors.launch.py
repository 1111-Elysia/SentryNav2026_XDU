from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    pkg_bringup = FindPackageShare('bringup').find('bringup')
    pkg_sentry = FindPackageShare('sentry_navigation').find('sentry_navigation')
    pkg_livox_to_scan = FindPackageShare('livox_to_scan').find('livox_to_scan')
    pkg_lightning = FindPackageShare('lightning-lm').find('lightning-lm')

    # 参数文件绝对路径
    tf_params = os.path.join(pkg_sentry, 'config', 'lidar.yaml')
    livox_scan_params = os.path.join(pkg_livox_to_scan, 'config', 'livox_to_scan_params.yaml')
    lightning_config = os.path.join(pkg_lightning, 'config', 'default_livox.yaml')

    # 节点启动
    livox = ExecuteProcess(
        cmd=["ros2", "launch", "livox_ros_driver2", "msg_MID360_launch.py"],
        output="screen"
    )

    lightning = ExecuteProcess(
        cmd=["ros2", "run", "lightning", "run_loc_online", "--config", lightning_config],
        output="screen"
    )

    fast_lio = ExecuteProcess(
        cmd=["ros2", "launch", "fast_lio", "mapping.launch.py"],
        output="screen"
    )

    tf_pub = ExecuteProcess(
        cmd=[
            "ros2", "run", "sentry_navigation", "tf_odom_publisher",
            "--ros-args",
            "--params-file", tf_params
        ],
        output="screen"
    )

    livox_to_scan = ExecuteProcess(
        cmd=[
            "ros2", "run", "livox_to_scan", "livox_to_scan_node",
            "--ros-args",
            "--params-file", livox_scan_params
        ],
        output="screen"
    )

    return LaunchDescription([
        livox,
        TimerAction(period=3.0, actions=[lightning]),
        TimerAction(period=5.0, actions=[fast_lio]),
        TimerAction(period=7.0, actions=[tf_pub]),
        TimerAction(period=8.0, actions=[livox_to_scan]),
    ])

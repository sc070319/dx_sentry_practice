# auto_save_map.launch.py
import os
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess

def generate_launch_description():
    ld = LaunchDescription()

    # 路径自动从本文件位置推导(realpath 解析符号链接), 换机器/换目录不用再手改
    launch_dir = os.path.dirname(os.path.realpath(__file__))            # .../src/cod_bringup/launch
    package_dir = os.path.dirname(launch_dir)                           # .../src/cod_bringup
    ws_root = os.path.abspath(os.path.join(package_dir, '..', '..', '..', '..'))   # 工作区根目录
    save_dir = os.path.join(package_dir, 'maps', 'auto_save')

    def create_save_command(suffix: str) -> list:
        return [
            'bash', '-c',
            'source /opt/ros/humble/setup.bash && '
            f'source {os.path.join(ws_root, "install", "setup.bash")} && '
            f'mkdir -p {save_dir} && '
            f'ros2 run nav2_map_server map_saver_cli -f {save_dir}/auto_map_{suffix}'
        ]

    intervals = [ 30, 60, 90, 120, 150, 180, 210, 240, 270, 300]    #保存时间间隔
    for t in intervals:
        action = TimerAction(
            period=float(t),
            actions=[ExecuteProcess(cmd=create_save_command("$(date +%H%M%S)"), output='screen')]
        )
        ld.add_action(action)

    return ld
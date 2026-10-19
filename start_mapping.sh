#!/usr/bin/env bash
# start_mapping.sh —— 一键干净启动建图（先清理所有旧实例）
#
# 用法:  ./start_mapping.sh
# 做的事:
#   1) 停掉旧的 singlenav / multiplenav / 雷达驱动 / LIO / slam / TF 相关进程
#   2) 确认雷达驱动（nav_MID360_launch.py，PointCloud2，无 RViz）在跑，没有则启动
#   3) 启动 multiplenav_launch.py
#
# 注意: 会丢弃当前未保存的建图结果，需要先保存请先执行 ./save_map.sh
set -uo pipefail

WS="$HOME/Desktop/COD26_ws"
PATTERNS='singlenav_launch.py|multiplenav_launch.py|nav_MID360_launch.py|msg_MID360_launch.py|rviz_MID360_launch.py|livox_ros_driver2_node|small_point_lio|yaw_tf_broadcaster|fake_vel_transform|auto_level_map|async_slam_toolbox|pointcloud_to_laserscan|lidar_filter_node|nav2_lifecycle_manager|nav2_controller|nav2_planner|nav2_bt_navigator|nav2_map_server|nav2_behaviors|nav2_smoother|nav2_waypoint|nav2_velocity_smoother|cod_serial_ul26/cod_serial|rviz2 -d .*cod_nav'

echo "==== 1. 停止旧实例 ===="
pkill -TERM -f "$PATTERNS" 2>/dev/null || true
sleep 4
pkill -9 -f "$PATTERNS" 2>/dev/null || true
sleep 1
echo "  剩余: $(pgrep -f "$PATTERNS" | wc -l) 个"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

echo "==== 2. 雷达驱动 ===="
if pgrep -f 'livox_ros_driver2_node' >/dev/null; then
    echo "  已在运行，复用"
else
    echo "  启动 nav_MID360_launch.py (PointCloud2, 无 RViz)"
    nohup ros2 launch livox_ros_driver2 nav_MID360_launch.py > /tmp/livox_nav.log 2>&1 &
    sleep 6
    if pgrep -f 'livox_ros_driver2_node' >/dev/null; then
        echo "  驱动已启动"
    else
        echo "  ❌ 驱动启动失败，查看 /tmp/livox_nav.log"; exit 1
    fi
fi

echo "==== 3. 启动建图 ===="
echo "  提示: 启动后前 12 秒不要移动机器人（auto_level_map 要测地面）"
cd "$WS"
exec ros2 launch cod_bringup multiplenav_launch.py

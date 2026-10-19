#!/usr/bin/env bash
# 一键运行手柄学习向导 (自动加载 ROS 环境, 任意终端可用)
cd "$HOME/Desktop/COD26_ws" || exit 1
source /opt/ros/humble/setup.bash
source "$HOME/Desktop/COD26_ws/install/setup.bash" 2>/dev/null
exec python3 -u joy_map_wizard.py

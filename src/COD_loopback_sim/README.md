# Nav2 LoopBack Sim
A lightweight testing environment designed for rapidly validating algorithms and converting them into Nav2 plugins.

Using Gazebo for initial path planning and control testing can be unnecessarily cumbersome. It forces developers to contend with physics-engine artifacts—such as odometry drift and positioning errors—which are distractions at this stage. Loopback isolates the control logic, allowing us to focus purely on algorithm verification before transitioning to high-fidelity simulations.

## Motivation

The official nav2_loopback_sim is readily available for Jazzy. However, since it hasn't been backported to the Humble apt sources yet, I created this standalone integration to provide equivalent functionality on Humble.

> **环境说明**：本项目原为 macOS (RoboStack + Pixi) 版本，现已适配 **Ubuntu 22.04 + ROS 2 Humble（原生 apt 安装）**，不再依赖 Pixi/RoboStack。

## Prerequisites
- Ubuntu 22.04 + ROS 2 Humble (desktop)
- nav2 相关包（`ros-humble-navigation2`、`ros-humble-nav2-bringup`）
- `python3-transforms3d`、`ros-humble-tf-transformations`

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
                 ros-humble-tf-transformations python3-transforms3d
```

## How to build
```bash
cd ~/COD26_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select nav2_loopback_sim nav2_minimal_tb3_sim
source install/setup.bash
```

## How to run
```bash
source install/setup.bash
ros2 launch nav2_loopback_sim tb3_loopback_simulation_launch.py
```

启动参数（可选）：
```bash
ros2 launch nav2_loopback_sim tb3_loopback_simulation_launch.py \
    map:=/home/sc/COD26_ws/src/COD_loopback_sim/nav2_loopback_sim/maps/depot.yaml \
    use_rviz:=True
```

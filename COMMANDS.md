# COD26 操作速查（2026-09-19 定版）

> 新终端自动 source 工作区（已配好）；若报 "Package not found" → `exec bash` 或新开终端。
> 串口**固定路径**（会随插拔变编号，认准这个）：
> `/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_374A39593233-if00`

---

## 一、启动行为树

### 1. 先起导航链路（行为树要用 Nav2 的动作服务器）

```bash
# 终端1 · 雷达驱动（无屏用 msg 版；接屏幕调试用 rviz 版）
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# 终端2 · 导航栈（自动加载 maps/latest.yaml；无屏加 rviz:=false）
ros2 launch cod_bringup singlenav_launch.py
# 临时换地图: ros2 launch cod_bringup singlenav_launch.py map:=/绝对/路径/xxx.yaml
```

### 2. 启动行为树（⚠️ 必须先 cd 到行为树仓库目录！树路径是相对路径）

```bash
cd ~/Desktop/COD26_ws/src/cod_-rm2026_-behavior-tree

ros2 launch cod_behavior cod.launch.py            # 单点树 singlenav_tree.xml + cod_pose.yaml
# ros2 launch cod_behavior multiplenav.launch.py  # multiplenav_basic.xml
# ros2 launch cod_behavior test.launch.py         # 测试树 test.xml
```

**替代法（跳过旧串口节点，推荐）：**
```bash
cd ~/Desktop/COD26_ws/src/cod_-rm2026_-behavior-tree
ros2 run cod_behavior tree_1 --ros-args \
  --params-file ~/Desktop/COD26_ws/install/cod_behavior/share/cod_behavior/launch/cod_pose.yaml
```

### 注意事项
- ⚠️ `cod.launch.py` 同时会拉起**旧版 cod_serial**（写死 `/dev/ttyACM0`、旧协议）——
  和导航栈的 `cod_serial_ul26` 抢串口（设备若枚举成 ttyACM0 会互相踩）。目前建议用上面的"替代法"，
  或让 AI 把三个 launch 里的 serial_node 去掉。
- 行为树动作依赖：`/navigate_to_pose`、`/follow_waypoints`、`/navigate_through_poses`（导航栈起来才有）。
- 裁判数据 `rm_interfaces/SerialReceiveData` 目前**没有数据源**（新电控协议尚未对接）→ 依赖血量的分支不会触发。
- Groot2 在线可视化：端口 **5555**（Groot2 软件连 localhost:5555 看树状态）。
- **改航点**：编辑 `COD_Behavior/launch/cod_pose.yaml`（nav_pose.home / main / main1、
  D_patrol_pose / A_patrol_pose 坐标）；巡逻 CSV 在 `COD_Behavior/launch/*.csv`。

---

## 二、启动遥控哨兵

### 方式 A · 手柄（推荐）
```bash
ros2 launch pb_teleop_twist_joy pb_teleop_twist_joy_launch.py
# 手柄不是 js0 时: 加 joy_dev:=1

# 按键：按住 L1 使能 | 左摇杆=前后/平移 | 十字键=旋转 | R1=涡轮
```

### 方式 B · 键盘
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# 按键: i 前进 · , 后退 · j/l 左/右转 · k 急停 · q/z 加减速
```

### 遥控需要什么在跑？
- **导航/建图时遥控**：`multiplenav` 或 `singlenav` 已经包含串口+速度转换，直接开遥控即可。
- **纯遥控测试**（不跑导航、不跑雷达）——三个终端：
```bash
# 终端1 · 串口（带发送调试打印）
ros2 run cod_serial_ul26 cod_serial --ros-args \
  -p port:=/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_374A39593233-if00 \
  -p debug_tx:=true

# 终端2 · 速度坐标转换（没跑 LIO 时角度=0，指令原样透传）
ros2 run fake_vel_transform fake_vel_transform_node

# 终端3 · 手柄或键盘（见上）
```

### 验证
```bash
ros2 topic echo /cmd_vel       # 遥控输出
ros2 topic echo /aft_cmd_vel   # 转换后发往串口的
ros2 topic hz /yaw             # 电控上行 ~440Hz
```
⚠️ 遥控和 Nav2 都发 `/cmd_vel`：**不要一边遥控一边下发导航目标**。

---

## 三、与电控联调排查（按顺序查）

```bash
# 1) 设备在不在
ls -l /dev/ttyACM* ; ls -l /dev/serial/by-id/
groups | grep -o dialout        # 权限（应在 dialout 组）

# 2) 串口节点抓在哪个口上（重点：有没有抱死旧编号）
pgrep -af cod_serial
for p in $(pgrep -f cod_serial); do echo "PID $p:"; \
  ls -l /proc/$p/fd | grep -oE "/dev/tty[A-Za-z0-9]+" | sort -u; done

# 3) 上行（电控 → PC）
ros2 topic hz /yaw              # 期望 ~440Hz
ros2 topic echo /yaw --once

# 4) 下行链路连接关系
ros2 topic info /aft_cmd_vel --verbose    # Publisher/Subscription 数量、节点名

# 5) 手动发速度（车会动！悬空或确认空间）
ros2 topic pub -r 10 /aft_cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.25}}" --times 40
# debug_tx 打开时应看到:
# TX 0xA5: vx=+0.250 ... [A5 00 00 80 3E 00 00 00 00 00 00 00 00 63 01]

# 6) 重启串口节点（换设备号/卡住时）
pkill -f "cod_serial_ul26/lib"
ros2 run cod_serial_ul26 cod_serial --ros-args \
  -p port:=/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_374A39593233-if00 \
  -p debug_tx:=true
# 期望启动打印: UART opened: /dev/ttyACMn @115200

# 7) 看电控上行原始字节（可选；先停掉串口节点再执行）
stty -F /dev/ttyACM1 raw 115200 && timeout 2 cat /dev/ttyACM1 | xxd | head -5
```

### 常见故障对照表
| 现象 | 原因 | 处理 |
|---|---|---|
| 没有 `/dev/ttyACM*` | USB 线/电控没上电 | 检查硬件 |
| 节点 fd 指向不存在的 ttyACMn | 设备重枚举（0↔1 跳变）| 用 by-id 路径重启节点 |
| 没有 `/yaw` | 电控没在发 / 插错口 | 换口试 / 问电控 |
| `/aft_cmd_vel` 有订阅但没 TX 打印 | 话题/节点没对上 | `debug_tx:=true` 重启节点看 |
| TX 正常但电控不执行 | 校验和/协议版本 | 核对下方协议常量 |

---

## 附 · 关键常量（协议核对用）

- **下行速度帧**（PC→电控）15 字节：
  `[0]=0xA5 | [1:5]=vx | [5:9]=vy | [9:13]=vz (float32 小端) | [13:15]=sum(byte0..12)&0xFFFF 小端`
  示例 vx=0.25：`A5 00 00 80 3E 00 00 00 00 00 00 00 00 63 01`
- **上行帧**（电控→PC）29 字节：`'S','P'`(0x53 0x50) 头，yaw float32 小端在 **[3:7]**，
  CRC16（poly 0x8408 reflected / init 0xFFFF，覆盖 [0:27]）小端在 [27:29]。
- **速度链路**：遥控/Nav2 → `/cmd_vel` → `fake_vel_transform`（需 LIO 的 `/Odometry` 供角度）→ `/aft_cmd_vel` → `cod_serial_ul26` → 串口
- **地图**：`./save_map.sh` 保存（存 `maps/latest.*` + 归档）；导航默认加载 `maps/latest.yaml`
- **建图**：`ros2 launch cod_bringup multiplenav_launch.py`（无屏 `rviz:=false`）

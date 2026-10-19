#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""大疆遥控器(NDJ6)按键学习向导

用法:
    python3 joy_map_wizard.py      (需要 joy_node 在跑, /joy 有数据)
按屏幕提示逐个操作, 全程约 1 分钟。结束后把"测量结果汇总"整段发回。

注意: 每个动作做到底并保持 1 秒左右, 做完等屏幕提示再操作下一个。
"""
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

STEPS = [
    ("把【右摇杆】向上推到底, 保持住", "axis"),
    ("松开, 再把【右摇杆】向下推到底, 保持住", "axis"),
    ("松开, 把【右摇杆】向左推到底, 保持住", "axis"),
    ("松开, 把【右摇杆】向右推到底, 保持住", "axis"),
    ("松开, 把【左摇杆】向左推到底, 保持住", "axis"),
    ("松开, 把【左摇杆】向右推到底, 保持住", "axis"),
    ("把【左摇杆】垂直方向: 先推到最高, 再拉到最低", "axis"),
    ("把【S1】开关拨上再拨下(来回一次)", "button"),
    ("把【S2】开关拨上再拨下(来回一次)", "button"),
    ("按一下你想用作【使能键】的按键(录像或拍照键)", "button"),
]

NAMES = ['右摇杆-上', '右摇杆-下', '右摇杆-左', '右摇杆-右',
         '左摇杆-左', '左摇杆-右', '左摇杆-垂直(油门)', 'S1', 'S2', '使能键']


class Wizard(Node):
    def __init__(self):
        super().__init__('joy_map_wizard')
        self.latest = None
        self.create_subscription(Joy, 'joy', self.cb, 10)

    def cb(self, msg):
        self.latest = msg


def wait_joy(node, timeout=60.0):
    t0 = time.time()
    last_tip = 0.0
    while node.latest is None:
        now = time.time()
        if now - t0 > timeout:
            print('')
            print('!! 60 秒内没有收到 /joy 数据。请检查:')
            print('   1) 遥控器电源开关打开了吗? (指示灯亮)')
            print('   2) joy_node 在跑吗?    ros2 node list | grep joy')
            print('   3) 手动测一下:          ros2 topic hz /joy')
            sys.exit(1)
        if now - last_tip > 5:
            print('   ... 等待 /joy 数据中。请确认遥控器已【开机】')
            print('       (大疆遥控器: 长按电源键2秒开机; 插着充电线时无法开机)')
            last_tip = now
        time.sleep(0.1)
    print('   √ 收到 /joy 数据流')


def run_axis_step(node, prompt, timeout=90.0):
    print('\n>> ' + prompt)
    base = list(node.latest.axes)
    t0 = time.time()
    while time.time() - t0 < timeout:
        axes = list(node.latest.axes)
        best_i, best_d = -1, 0.0
        for i, (b, c) in enumerate(zip(base, axes)):
            if abs(c - b) > abs(best_d):
                best_d, best_i = c - b, i
        if best_i >= 0 and abs(best_d) >= 0.55:
            print('   √ 检测到: axis[%d]  = %+0.2f  (变化 %+0.2f)'
                  % (best_i, axes[best_i], best_d))
            return ('axis', best_i, axes[best_i])
        time.sleep(0.02)
    print('   × 超时, 这个方向没检测到变化!')
    return ('axis', -1, 0.0)


def run_button_step(node, prompt, timeout=90.0):
    print('\n>> ' + prompt)
    base = list(node.latest.buttons)
    t0 = time.time()
    found = []
    hit_time = None
    while time.time() - t0 < timeout:
        btns = list(node.latest.buttons)
        for i, (b, c) in enumerate(zip(base, btns)):
            if b != c:
                msg = '   √ 检测到: button[%d]  %d -> %d' % (i, b, c)
                if msg not in found:
                    print(msg)
                    found.append(msg)
                hit_time = time.time()
        if hit_time is not None and time.time() - hit_time > 1.5:
            break
        time.sleep(0.02)
    if not found:
        print('   × 超时, 没检测到按键变化!')
        return ('button', -1)
    idxs = [int(f.split('[')[1].split(']')[0]) for f in found]
    return ('button', idxs[0])


def precheck(node, timeout=25.0):
    """信号预检: 确认遥控器真的在发数据(而不是只有接收器的空数据流)"""
    print('\n== 信号预检: 请把【右摇杆】上下左右随便推一推 ==')
    base_axes = list(node.latest.axes)
    base_btn = list(node.latest.buttons)
    t0 = time.time()
    while time.time() - t0 < timeout:
        axes = list(node.latest.axes)
        btns = list(node.latest.buttons)
        moved = [i for i, (a, b) in enumerate(zip(base_axes, axes))
                 if abs(a - b) > 0.3]
        pressed = [i for i, (a, b) in enumerate(zip(base_btn, btns)) if a != b]
        if moved or pressed:
            print('   √ 收到遥控器信号! (变化轴: %s, 变化按键: %s)' % (moved, pressed))
            return True
        time.sleep(0.05)
    print('   × /joy 数据流是通的, 但遥控器没有任何信号进来。请检查:')
    print('     1) 遥控器【开机】了吗? (长按电源键2秒, 指示灯常亮)')
    print('     2) 遥控器是不是正插着充电线? (充电时无法开机使用)')
    print('     3) 遥控器靠近接收器一点(1~2米内), 重启遥控器再试')
    print('     4) 电量是否足够? 低电量先充电')
    return False


def main():
    rclpy.init()
    node = Wizard()
    print('=' * 54)
    print(' 大疆遥控器 按键学习向导')
    print(' 准备: 遥控器已开机、接收器插好')
    print('=' * 54)
    wait_joy(node)
    if not precheck(node):
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(2)
    print(' 开始逐个操作(动作做到底保持约1秒), 约1分钟')
    print('=' * 54)
    results = []
    for prompt, kind in STEPS:
        if kind == 'axis':
            results.append(run_axis_step(node, prompt))
        else:
            results.append(run_button_step(node, prompt))

    print('\n' + '=' * 54)
    print(' 测量结果汇总 (把下面整段发回来):')
    print('-' * 54)
    for nm, r in zip(NAMES, results):
        if r[0] == 'axis':
            if r[1] >= 0:
                print('  %-16s -> axis[%d]   值 %+0.2f' % (nm, r[1], r[2]))
            else:
                print('  %-16s -> 未检测到' % nm)
        else:
            if r[1] >= 0:
                print('  %-16s -> button[%d]' % (nm, r[1]))
            else:
                print('  %-16s -> 未检测到' % nm)
    print('=' * 54)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

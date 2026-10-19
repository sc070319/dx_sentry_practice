#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""手柄学习器: 实时显示你按了哪个键/动了哪根轴, 用来把真实编号测出来

用法:
    python3 joy_watch.py
 然后【一个一个】操作手柄控件(每个动作停留1秒):
    L1 → L2 → R1 → R2 → 十字键左/右/上/下 →
    左摇杆 前/后/左/右 → 右摇杆 前/后/左/右
  把打印出来的编号对照关系发出来, 即可修配置。
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoyWatch(Node):
    def __init__(self):
        super().__init__('joy_watch')
        self.prev_btn = None
        self.shown_axes = None       # 上次"显示过"的轴值, 防漂移刷屏
        self.create_subscription(Joy, 'joy', self.cb, 10)

    def cb(self, msg):
        axes = [round(a, 2) for a in msg.axes]
        btns = list(msg.buttons)

        if self.prev_btn is None:
            self.prev_btn = btns
            self.shown_axes = list(axes)
            print('=' * 46)
            print('检测到手柄: %d 根轴(axis 0~%d), %d 个按键(button 0~%d)'
                  % (len(axes), len(axes) - 1, len(btns), len(btns) - 1))
            print('现在开始逐个操作: L1 L2 R1 R2 | 十字键左右上下 | 左右摇杆')
            print('=' * 46)
            return

        # 按键变化
        for i, (p, c) in enumerate(zip(self.prev_btn, btns)):
            if p != c:
                print('  按键 button[%d]: %d -> %d   [%s]'
                      % (i, p, c, '按下' if c else '松开'))

        # 轴变化(与上次显示值差 >= 0.25 才打印)
        for i, (p, c) in enumerate(zip(self.shown_axes, axes)):
            if abs(c - p) >= 0.25:
                print('  轴   axis[%d]: %+.2f -> %+.2f' % (i, p, c))
                self.shown_axes[i] = c

        self.prev_btn = btns


def main():
    rclpy.init()
    node = JoyWatch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n退出')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

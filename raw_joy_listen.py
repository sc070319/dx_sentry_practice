#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""直接监听 /dev/input/js0 原始事件(绕过 ROS/joy_node)

用途: 判断遥控器到底有没有把信号发到 Jetson
  - 打印出 AXIS/BTN 行  = 信号正常, 问题在 ROS 层
  - 一直没有任何输出    = 遥控器没发信号(没开机/没对频/接收器问题)

用法: python3 raw_joy_listen.py [监听秒数, 默认300]
"""
import os
import select
import struct
import sys
import time

secs = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0

fd = os.open('/dev/input/js0', os.O_RDONLY | os.O_NONBLOCK)
print('监听 /dev/input/js0 原始事件, 最长 %.0f 秒...' % secs)
print('现在请: 推一推右摇杆 + 按几个按键')
print('-' * 46)

t0 = time.time()
cnt = 0
while time.time() - t0 < secs:
    r, _, _ = select.select([fd], [], [], 1.0)
    if not r:
        continue
    try:
        d = os.read(fd, 8)
    except BlockingIOError:
        continue
    if len(d) < 8:
        continue
    _t, val, typ, num = struct.unpack('IhBB', d)
    if typ & 0x80:          # 初始化事件, 忽略
        continue
    kind = 'AXIS' if typ == 2 else ('BTN ' if typ == 1 else 'EVT ')
    print('  %s [%d] = %d' % (kind, num, val))
    cnt += 1

print('-' * 46)
print('监听结束, 共收到 %d 个事件' % cnt)
if cnt == 0:
    print('>>> 一个事件都没收到: 遥控器信号没到 Jetson')

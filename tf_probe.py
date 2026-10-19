#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""查询两个坐标系之间的 TF，打印平移 + RPY(度)

用法:
    python3 tf_probe.py                      # 默认 map -> base_link
    python3 tf_probe.py <target> <source>    # 例如 python3 tf_probe.py map livox_frame
"""
import math
import sys
import time

import rclpy
from tf2_ros import Buffer, TransformListener


def quat_to_rpy(x, y, z, w):
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def main():
    target = sys.argv[1] if len(sys.argv) > 1 else 'map'
    source = sys.argv[2] if len(sys.argv) > 2 else 'base_link'
    hold = float(sys.argv[3]) if len(sys.argv) > 3 else 8.0

    rclpy.init()
    node = rclpy.create_node('tf_probe')
    buf = Buffer()
    TransformListener(buf, node)

    t0 = time.time()
    tr = None
    while time.time() - t0 < hold:
        rclpy.spin_once(node, timeout_sec=0.2)
        try:
            tr = buf.lookup_transform(target, source, rclpy.time.Time())
            break
        except Exception:
            continue

    if tr is None:
        print('拿不到 %s -> %s 的 TF (%.0f 秒内) —— 检查 TF 树是否连通' % (target, source, hold))
    else:
        t = tr.transform.translation
        q = tr.transform.rotation
        r, p, y = quat_to_rpy(q.x, q.y, q.z, q.w)
        print('%s -> %s :' % (target, source))
        print('  平移 [%.3f, %.3f, %.3f]' % (t.x, t.y, t.z))
        print('  姿态 RPY(度) = (%.2f, %.2f, %.2f)'
              % (math.degrees(r), math.degrees(p), math.degrees(y)))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""根据地平面自动测量 map->odom 需要的补偿角

原理: 取 LIO 配准后的点云, 拟合地平面法向量, 反推让 map 系水平所需的旋转。
       比"按装位角度硬补偿"更准 —— 连"车/地面本身有坡度"一起修正。

用法(需建图栈在跑、车静止):
    python3 measure_map_tilt.py [点云话题, 默认 /cloud_registered]
输出:
    - 地面法向量在该坐标系下的姿态
    - 建议的 map->odom 补偿角(roll/pitch, 弧度), 直接填进 launch
"""
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rotation_from_to(a, b):
    """返回把单位向量 a 转到单位向量 b 的最小旋转矩阵"""
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    if np.linalg.norm(v) < 1e-8:
        return np.eye(3) if c > 0 else -np.eye(3)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * (1.0 / (1.0 + c))


def mat_to_rpy(R):
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


class Probe(Node):
    def __init__(self, topic):
        super().__init__('measure_map_tilt')
        self.msg = None
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointCloud2, topic, self.cb, qos)
        self.create_subscription(PointCloud2, topic, self.cb, 10)
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)

    def cb(self, msg):
        if self.msg is None:
            self.msg = msg


def field_offsets(msg):
    off = {}
    for f in msg.fields:
        off[f.name] = f.offset
    return off


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else '/cloud_registered'
    rclpy.init()
    node = Probe(topic)

    t0 = node.get_clock().now()
    while node.msg is None:
        rclpy.spin_once(node, timeout_sec=0.2)
        if (node.get_clock().now() - t0).nanoseconds > 8e9:
            print('8 秒内没收到 %s 的点云 —— 建图栈在跑吗?' % topic)
            return

    msg = node.msg
    off = field_offsets(msg)
    data = np.frombuffer(msg.data, dtype=np.uint8)
    step = msg.point_step
    n = msg.width * msg.height
    arr = data[:n * step].reshape(n, step)

    def col(name):
        o = off.get(name)
        if o is None:
            return None
        return arr[:, o:o + 4].copy().view('<f4').ravel().astype(np.float64)

    x, y, z = col('x'), col('y'), col('z')
    pts = np.c_[x, y, z]
    good = np.isfinite(pts).all(axis=1)
    pts = pts[good]

    # 转到 odom 系 (地平面和重力方向在 odom 系里考察)
    frame = msg.header.frame_id
    print('点云话题: %s   坐标系: %s   点数: %d' % (topic, frame, len(pts)))
    tr = None
    t1 = time.time()
    while time.time() - t1 < 8.0:
        rclpy.spin_once(node, timeout_sec=0.2)
        try:
            tr = node.buf.lookup_transform('odom', frame, rclpy.time.Time())
            break
        except Exception:
            continue
    if tr is None:
        print('8 秒内拿不到 odom -> %s 的 TF, 检查 TF 树' % frame)
        return
    t = tr.transform.translation
    q = tr.transform.rotation
    R_tf = rot_x(0.0)  # placeholder, 下面用四元数构造
    # 四元数 -> 矩阵
    xx, yy, zz, ww = q.x, q.y, q.z, q.w
    R_tf = np.array([
        [1 - 2 * (yy * yy + zz * zz), 2 * (xx * yy - zz * ww), 2 * (xx * zz + yy * ww)],
        [2 * (xx * yy + zz * ww), 1 - 2 * (xx * xx + zz * zz), 2 * (yy * zz - xx * ww)],
        [2 * (xx * zz - yy * ww), 2 * (yy * zz + xx * ww), 1 - 2 * (xx * xx + yy * yy)]])
    pts = pts @ R_tf.T + np.array([t.x, t.y, t.z])

    # 取 2~6m 环带的地面点(避开车身和远处噪点)
    r = np.hypot(pts[:, 0], pts[:, 1])
    sel = (r > 2.0) & (r < 6.0)
    p = pts[sel]
    if len(p) < 200:
        print('地面点太少(%d), 换个话题或看看雷达是否正常' % len(p))
        return

    # RANSAC 拟合主平面(地面), 抗噪点/杂物干扰
    rng = np.random.default_rng(0)
    best = None
    for _ in range(400):
        idx = rng.choice(len(p), 3, replace=False)
        p0, p1, p2 = p[idx]
        nrm = np.cross(p1 - p0, p2 - p0)
        nn = np.linalg.norm(nrm)
        if nn < 1e-6:
            continue
        nrm = nrm / nn
        d = -float(nrm @ p0)
        inl = np.abs(p @ nrm + d) < 0.03
        if best is None or inl.sum() > best.sum():
            best = inl
    if best is None or best.sum() < 300:
        print('没找到足够大的地平面(内点 %s) —— 地面被遮挡或雷达异常?'
              % (0 if best is None else int(best.sum())))
        return
    q = p[best]
    cen = q.mean(axis=0)
    _, _, Vt = np.linalg.svd(q - cen)
    nvec = Vt[2]
    if nvec[2] < 0:
        nvec = -nvec
    resid = np.abs((q - cen) @ nvec)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, nvec[2]))))
    dist = float(nvec @ q.mean(axis=0))  # 平面到odom原点距离(参考)

    print('地面拟合: 内点 %d / 候选 %d, 残差 std %.3f m, 平面距原点 %.2f m'
          % (int(best.sum()), len(p), resid.std(), dist))
    print('地面法向量(odom系) = [%.4f, %.4f, %.4f]   地面倾斜 %.2f°' % (nvec[0], nvec[1], nvec[2], tilt))

    # 需要把法向量转到 +z 的旋转 = map->odom 补偿
    R_fix = rotation_from_to(nvec, np.array([0.0, 0.0, 1.0]))
    roll, pitch, yaw = mat_to_rpy(R_fix)
    print('')
    print('==> 建议 map->odom 静态变换改成:')
    print('     --roll  %.4f' % roll)
    print('     --pitch %.4f' % pitch)
    print('     --yaw   %.4f   (保持 0, 只是分解产物)' % yaw)
    print('    (度数: roll %.2f°, pitch %.2f°, yaw %.2f°)' % (
        math.degrees(roll), math.degrees(pitch), math.degrees(yaw)))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""auto_level_map —— 自动测量并发布 map->odom，让地图坐标系始终水平

背景:
    LIO(small_point_lio) 的世界系(odom) = "启动瞬间的 IMU 姿态"，因此相对水平面
    是倾斜的，而且**每次启动的倾角都可能不同**（实测见过 22.5° 和 28.8°）。
    把补偿角写死在 launch 里会时好时坏；本节点改为：启动后用地面点云实测补偿角，
    然后持续发布 map->odom（替代原来的静态 map->odom）。

行为:
    启动后 measure_delay 秒开始测量（等 LIO 稳定），每隔 sample_gap 秒测一次，
    取 3 次的平均（SVD 正交化）后**冻结**（避免建图中途跳变），之后 50Hz 持续发布。
    测量期间（约20秒）请勿移动机器人。

参数:
    cloud_topic    点云话题            (默认 /livox/lidar)
    map_frame      (默认 map)          odom_frame (默认 odom)
    initial_roll   测量前的兜底补偿角   (默认 0.3839 ≈22°)
    measure_delay  启动后延迟测量秒数   (默认 12s)
    sample_gap     采样间隔            (默认 3s)
    ring_min/max   地面点环带(m)        (默认 2.0 / 6.0)
    yaw_offset     额外绕 map z 旋转    (默认 0.0; singlenav 对旧图用 -0.5)
    z_offset       map->odom 的 z      (默认 0.05)
"""
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rotation_from_to(a, b):
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    if np.linalg.norm(v) < 1e-8:
        return np.eye(3) if c > 0 else -np.eye(3)
    vx = np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
    return np.eye(3) + vx + vx @ vx * (1.0 / (1.0 + c))


def mat_to_rpy(R):
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def mat_to_quat(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2.0
        return ((R[2, 1] - R[1, 2]) / S, (R[0, 2] - R[2, 0]) / S,
                (R[1, 0] - R[0, 1]) / S, 0.25 * S)
    if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        return (0.25 * S, (R[0, 1] + R[1, 0]) / S, (R[0, 2] + R[2, 0]) / S,
                (R[2, 1] - R[1, 2]) / S)
    if R[1, 1] > R[2, 2]:
        S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        return ((R[0, 1] + R[1, 0]) / S, 0.25 * S, (R[1, 2] + R[2, 1]) / S,
                (R[0, 2] - R[2, 0]) / S)
    S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
    return ((R[0, 2] + R[2, 0]) / S, (R[1, 2] + R[2, 1]) / S, 0.25 * S,
            (R[1, 0] - R[0, 1]) / S)


class AutoLevelMap(Node):
    def __init__(self):
        super().__init__('auto_level_map')
        for name, default in (('cloud_topic', '/livox/lidar'), ('map_frame', 'map'),
                              ('odom_frame', 'odom'), ('initial_roll', 0.3839),
                              ('measure_delay', 12.0), ('sample_gap', 3.0),
                              ('ring_min', 2.0), ('ring_max', 6.0),
                              ('yaw_offset', 0.0), ('z_offset', 0.05),
                              ('publish_tf', True)):
            self.declare_parameter(name, default)

        self.publish_tf_enabled = bool(self.get_parameter('publish_tf').value)

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.ring_min = float(self.get_parameter('ring_min').value)
        self.ring_max = float(self.get_parameter('ring_max').value)
        self.yaw_offset = float(self.get_parameter('yaw_offset').value)
        self.z_offset = float(self.get_parameter('z_offset').value)
        self.sample_gap = float(self.get_parameter('sample_gap').value)

        roll0 = float(self.get_parameter('initial_roll').value)
        self.R = rot_z(self.yaw_offset) @ np.array(
            [[1, 0, 0], [0, math.cos(roll0), -math.sin(roll0)],
             [0, math.sin(roll0), math.cos(roll0)]])
        self.quat = mat_to_quat(self.R)

        self.cloud = None
        topic = self.get_parameter('cloud_topic').value
        self.create_subscription(
            PointCloud2, topic, self.on_cloud,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(PointCloud2, topic, self.on_cloud, 10)

        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.br = TransformBroadcaster(self)

        self.samples = []
        self.done = False
        self.next_time = time.time() + float(self.get_parameter('measure_delay').value)

        self.create_timer(0.02, self.publish_tf)     # 50 Hz
        self.create_timer(1.0, self.try_measure)
        self.get_logger().info(
            'auto_level_map 启动: %.0f 秒后开始测地面, 期间请勿移动机器人'
            % float(self.get_parameter('measure_delay').value))

    # ---------- 发布 map->odom ----------
    def publish_tf(self):
        if not self.publish_tf_enabled:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.z = self.z_offset
        x, y, z, w = self.quat
        t.transform.rotation.x = x
        t.transform.rotation.y = y
        t.transform.rotation.z = z
        t.transform.rotation.w = w
        self.br.sendTransform(t)

    # ---------- 测量 ----------
    def on_cloud(self, msg):
        self.cloud = msg

    def measure_once(self):
        msg = self.cloud
        offs = {f.name: f.offset for f in msg.fields}
        if not all(k in offs for k in ('x', 'y', 'z')):
            return None
        n = msg.width * msg.height
        arr = np.frombuffer(msg.data, dtype=np.uint8)[:n * msg.point_step] \
            .reshape(n, msg.point_step)

        def col(name):
            o = offs[name]
            return arr[:, o:o + 4].copy().view('<f4').ravel().astype(np.float64)

        pts = np.c_[col('x'), col('y'), col('z')]
        pts = pts[np.isfinite(pts).all(axis=1)]

        frame = msg.header.frame_id
        tr = None
        try:
            tr = self.buf.lookup_transform(self.odom_frame, frame, rclpy.time.Time())
        except Exception:
            return None
        q = tr.transform.rotation
        xx, yy, zz, ww = q.x, q.y, q.z, q.w
        R_tf = np.array([
            [1 - 2 * (yy * yy + zz * zz), 2 * (xx * yy - zz * ww), 2 * (xx * zz + yy * ww)],
            [2 * (xx * yy + zz * ww), 1 - 2 * (xx * xx + zz * zz), 2 * (yy * zz - xx * ww)],
            [2 * (xx * zz - yy * ww), 2 * (yy * zz + xx * ww), 1 - 2 * (xx * xx + yy * yy)]])
        t = tr.transform.translation
        pts = pts @ R_tf.T + np.array([t.x, t.y, t.z])

        r = np.hypot(pts[:, 0], pts[:, 1])
        p = pts[(r > self.ring_min) & (r < self.ring_max)]
        if len(p) < 500:
            return None

        # RANSAC 找主平面(地面)
        rng = np.random.default_rng(0)
        best = None
        for _ in range(300):
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
        if best is None or best.sum() < 500:
            return None
        q_pts = p[best]
        cen = q_pts.mean(axis=0)
        _, _, Vt = np.linalg.svd(q_pts - cen)
        nvec = Vt[2]
        if nvec[2] < 0:
            nvec = -nvec
        return rotation_from_to(nvec, np.array([0.0, 0.0, 1.0]))

    def try_measure(self):
        if self.done or time.time() < self.next_time:
            return
        if self.cloud is None:
            self.get_logger().warn('还没收到点云, 3 秒后重试')
            self.next_time = time.time() + 3.0
            return
        R = self.measure_once()
        if R is None:
            self.get_logger().warn('地面拟合失败(TF没好/点太少), 3 秒后重试')
            self.next_time = time.time() + 3.0
            return
        self.samples.append(R)
        self.get_logger().info('地面采样 %d/3 完成' % len(self.samples))
        if len(self.samples) >= 3:
            Rm = np.mean(np.array(self.samples), axis=0)
            U, _, Vt = np.linalg.svd(Rm)
            Rm = U @ Vt
            if np.linalg.det(Rm) < 0:
                U[:, -1] = -U[:, -1]
                Rm = U @ Vt
            self.R = rot_z(self.yaw_offset) @ Rm
            self.quat = mat_to_quat(self.R)
            roll, pitch, yaw = mat_to_rpy(self.R)
            self.done = True
            self.get_logger().info(
                'map->odom 自动水平补偿就绪: roll=%.2f° pitch=%.2f° yaw=%.2f°'
                % (math.degrees(roll), math.degrees(pitch), math.degrees(yaw)))
        else:
            self.next_time = time.time() + self.sample_gap


def main():
    rclpy.init()
    node = AutoLevelMap()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

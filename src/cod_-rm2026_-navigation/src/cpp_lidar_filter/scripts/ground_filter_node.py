#!/usr/bin/env python3
"""Remove the local floor before STVL applies its map-frame height limits."""

from array import array
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


def cloud_arrays(msg):
    fields = {field.name: field for field in msg.fields}
    if any(name not in fields or fields[name].datatype != PointField.FLOAT32
           for name in ('x', 'y', 'z')):
        raise ValueError('expected float32 x/y/z fields')
    dtype = np.dtype({
        'names': ['x', 'y', 'z'],
        'formats': [('>f4' if msg.is_bigendian else '<f4')] * 3,
        'offsets': [fields[name].offset for name in ('x', 'y', 'z')],
        'itemsize': msg.point_step,
    })
    shape = (msg.height, msg.width)
    strides = (msg.row_step, msg.point_step)
    xyz = np.ndarray(shape, dtype=dtype, buffer=msg.data, strides=strides)
    points = np.column_stack([xyz[name].ravel() for name in ('x', 'y', 'z')])
    records = np.ndarray(shape, dtype=f'V{msg.point_step}', buffer=msg.data,
                         strides=strides).ravel()
    return points.astype(np.float64), records


def fit_ground(points, distance, min_points, max_angle):
    """Return a plane in base_link, guarded by the measured chassis clearance."""
    radius = np.hypot(points[:, 0], points[:, 1])
    candidates = points[(radius > 0.4) & (radius < 3.0) &
                        (points[:, 2] > -0.5) & (points[:, 2] < 0.05)]
    if len(candidates) < min_points:
        return None
    rng = np.random.default_rng(0)
    if len(candidates) > 1500:
        candidates = candidates[rng.choice(len(candidates), 1500, replace=False)]
    min_normal_z = math.cos(math.radians(max_angle))
    best = None
    best_count = max(min_points - 1, int(len(candidates) * 0.35) - 1)
    for _ in range(50):
        a, b, c = candidates[rng.choice(len(candidates), 3, replace=False)]
        normal = np.cross(b - a, c - a)
        norm = np.linalg.norm(normal)
        if norm < 1e-8:
            continue
        normal /= norm
        if abs(normal[2]) < min_normal_z:
            continue
        offset = -float(normal @ a)
        # Ground was measured at base_link z approximately -0.16 to -0.26 m.
        if not -0.4 < -offset / normal[2] < -0.08:
            continue
        residual = np.einsum('ij,j->i', candidates, normal) + offset
        inliers = np.abs(residual) <= distance
        count = int(inliers.sum())
        if count > best_count:
            best, best_count = inliers, count
    if best is None:
        return None
    floor = candidates[best]
    center = floor.mean(axis=0)
    _, singular, vt = np.linalg.svd(floor - center, full_matrices=False)
    if singular[1] < 0.05:
        return None
    normal = vt[-1]
    if normal[2] < 0:
        normal = -normal
    offset = -float(normal @ center)
    if normal[2] < min_normal_z or not -0.4 < -offset / normal[2] < -0.08:
        return None
    return normal, offset


class GroundFilter(Node):
    def __init__(self):
        super().__init__('local_ground_filter')
        for name, default in (
                ('input_topic', '/livox/lidar_cropped'),
                ('output_topic', '/livox/lidar_filtered'),
                ('base_frame', 'base_link'), ('ground_distance', 0.035),
                ('min_ground_points', 200), ('max_ground_angle', 20.0),
                ('overhang_filter_enabled', False),
                ('overhang_min', [0.0, 0.0, 0.0]),
                ('overhang_max', [0.0, 0.0, 0.0])):
            self.declare_parameter(name, default)
        self.base_frame = self.get_parameter('base_frame').value
        self.distance = float(self.get_parameter('ground_distance').value)
        self.min_points = int(self.get_parameter('min_ground_points').value)
        self.max_angle = float(self.get_parameter('max_ground_angle').value)
        self.filter_overhang = bool(self.get_parameter('overhang_filter_enabled').value)
        self.overhang_min = np.asarray(self.get_parameter('overhang_min').value, dtype=float)
        self.overhang_max = np.asarray(self.get_parameter('overhang_max').value, dtype=float)
        if self.filter_overhang and (
                self.overhang_min.shape != (3,) or self.overhang_max.shape != (3,) or
                not np.isfinite(self.overhang_min).all() or
                not np.isfinite(self.overhang_max).all() or
                not (self.overhang_min < self.overhang_max).all()):
            raise ValueError('overhang bounds must contain three finite min < max values')
        self.publisher = self.create_publisher(
            PointCloud2, self.get_parameter('output_topic').value, 10)
        self.subscription = self.create_subscription(
            PointCloud2, self.get_parameter('input_topic').value,
            self.on_cloud, qos_profile_sensor_data)
        self.last_log = -math.inf
        self.last_warning = -math.inf

    def warn(self, message):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_warning >= 5.0:
            self.get_logger().warn(message)
            self.last_warning = now

    def on_cloud(self, msg):
        if msg.header.frame_id != self.base_frame:
            self.warn(f'Expected {self.base_frame}, got {msg.header.frame_id}; passing cloud through')
            self.publisher.publish(msg)
            return
        try:
            points, records = cloud_arrays(msg)
        except (ValueError, TypeError) as error:
            self.warn(f'Cannot decode cloud: {error}; passing cloud through')
            self.publisher.publish(msg)
            return
        finite = np.isfinite(points).all(axis=1)
        overhang = np.zeros(len(points), dtype=bool)
        if self.filter_overhang:
            overhang = finite & (points >= self.overhang_min).all(axis=1) & (
                points <= self.overhang_max).all(axis=1)
        plane = fit_ground(points[finite & ~overhang], self.distance,
                           self.min_points, self.max_angle)
        ground = np.zeros(len(points), dtype=bool)
        ground_height = 'unavailable'
        if plane is None:
            self.warn('No reliable local ground plane; applying only the body filter')
        else:
            normal, offset = plane
            residual = np.einsum('ij,j->i', points, normal) + offset
            ground = finite & (np.abs(residual) <= self.distance)
            ground_height = f'{-offset / normal[2]:.3f}m'
        keep = finite & ~ground & ~overhang
        output = PointCloud2()
        output.header = msg.header
        output.height = 1
        output.width = int(keep.sum())
        output.fields = msg.fields
        output.is_bigendian = msg.is_bigendian
        output.point_step = msg.point_step
        output.row_step = output.width * msg.point_step
        output.is_dense = True
        output.data = array('B', records[keep].tobytes())
        self.publisher.publish(output)
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_log >= 5.0:
            self.get_logger().info(
                f'ground_z={ground_height} ground_removed={int(ground.sum())} '
                f'body_removed={int(overhang.sum())}/{len(points)} kept={output.width}')
            self.last_log = now


def main():
    rclpy.init()
    node = GroundFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

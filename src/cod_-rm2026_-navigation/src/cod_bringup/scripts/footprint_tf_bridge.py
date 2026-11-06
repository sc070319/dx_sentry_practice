#!/usr/bin/env python3
"""Express the measured chassis footprint in Nav2's rotated base frame."""

import math

import rclpy
from geometry_msgs.msg import Point32, Polygon
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class FootprintTFBridge(Node):
    def __init__(self):
        super().__init__('physical_footprint_bridge')
        for name, default in (
                ('body_length', 0.52), ('body_width', 0.52),
                ('body_frame', 'base_link'), ('costmap_base_frame', 'base_link_fake'),
                ('local_topic', '/local_costmap/footprint'),
                ('global_topic', '/global_costmap/footprint')):
            self.declare_parameter(name, default)
        self.body_frame = self.get_parameter('body_frame').value
        self.target_frame = self.get_parameter('costmap_base_frame').value
        length = float(self.get_parameter('body_length').value)
        width = float(self.get_parameter('body_width').value)
        if length <= 0 or width <= 0:
            raise ValueError('body_length and body_width must be positive')
        hx, hy = length / 2, width / 2
        self.declare_parameter('body_footprint', [hx, hy, hx, -hy, -hx, -hy, -hx, hy])
        vertices = list(self.get_parameter('body_footprint').value)
        if len(vertices) < 6 or len(vertices) % 2 or not all(map(math.isfinite, vertices)):
            raise ValueError('body_footprint must contain at least three finite x/y pairs')
        self.corners = list(zip(vertices[::2], vertices[1::2]))
        area = sum(x * ny - nx * y for (x, y), (nx, ny) in
                   zip(self.corners, self.corners[1:] + self.corners[:1]))
        if abs(area) < 1e-6:
            raise ValueError('body_footprint must have a nonzero area')
        self.footprint_publishers = [
            self.create_publisher(Polygon, self.get_parameter(name).value, 1)
            for name in ('local_topic', 'global_topic')]
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.previous = None
        self.last_publish = -math.inf
        self.last_warning = -math.inf
        self.timer = self.create_timer(0.1, self.publish_footprint)

    def publish_footprint(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        try:
            tf = self.buffer.lookup_transform(self.target_frame, self.body_frame, Time())
        except TransformException as error:
            if now - self.last_warning >= 5.0:
                self.get_logger().warn(f'Waiting for chassis footprint TF: {error}')
                self.last_warning = now
            return
        q = tf.transform.rotation
        t = tf.transform.translation
        xx = 1 - 2 * (q.y * q.y + q.z * q.z)
        xy = 2 * (q.x * q.y - q.z * q.w)
        yx = 2 * (q.x * q.y + q.z * q.w)
        yy = 1 - 2 * (q.x * q.x + q.z * q.z)
        vertices = [(xx * x + xy * y + t.x, yx * x + yy * y + t.y)
                    for x, y in self.corners]
        # Avoid forcing full inflation recomputation for identical polygons.
        if self.previous is not None and now - self.last_publish < 2.0:
            change = max(math.hypot(x - px, y - py)
                         for (x, y), (px, py) in zip(vertices, self.previous))
            if change < 0.005:
                return
        polygon = Polygon()
        polygon.points = [Point32(x=float(x), y=float(y), z=0.0) for x, y in vertices]
        for publisher in self.footprint_publishers:
            publisher.publish(polygon)
        if self.previous is None:
            self.get_logger().info(
                f'Publishing physical chassis footprint in {self.target_frame}')
        self.previous = vertices
        self.last_publish = now


def main():
    rclpy.init()
    node = FootprintTFBridge()
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

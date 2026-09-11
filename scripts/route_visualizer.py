#!/usr/bin/env python3
"""Render the latest mission route as a Gazebo GUI marker."""

import os
import subprocess
import threading
import math

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile


class RouteVisualizer(Node):
    def __init__(self):
        super().__init__('route_visualizer')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._publish_lock = threading.Lock()
        self.create_subscription(Path, '/drone/mission_path', self.on_path, qos)
        self.get_logger().info('Gazebo route visualizer ready')

    def on_path(self, message):
        if len(message.poses) < 2:
            return
        route_points = []
        for pose in message.poses:
            route_points.append((
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z + 0.35,
            ))

        # Gazebo LINE_STRIP markers are always one pixel wide, and triangle
        # markers can disappear when their winding faces away from the camera.
        # Use thick box segments instead so every flight leg is visible from
        # the farm overview, including vertical takeoff and landing legs.
        segment_markers = []
        for index, (start, end) in enumerate(zip(route_points, route_points[1:]), start=1):
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            dz = end[2] - start[2]
            length = math.hypot(dx, dy)
            spatial_length = math.sqrt(dx * dx + dy * dy + dz * dz)
            if spatial_length < 1e-6:
                continue
            center = tuple((left + right) / 2.0 for left, right in zip(start, end))
            if length >= 1e-6:
                yaw = math.atan2(dy, dx)
                scale = f'x: {spatial_length:.3f} y: 0.8 z: 0.65'
                orientation = (
                    f'x: 0 y: 0 z: {math.sin(yaw / 2.0):.6f} '
                    f'w: {math.cos(yaw / 2.0):.6f}'
                )
            else:
                scale = f'x: 0.8 y: 0.8 z: {spatial_length:.3f}'
                orientation = 'x: 0 y: 0 z: 0 w: 1'
            segment_markers.append(' '.join([
                'action: ADD_MODIFY',
                'ns: "drone_demo_route"',
                f'id: {index}',
                'type: BOX',
                f'scale {{ {scale} }}',
                f'pose {{ position {{ x: {center[0]:.3f} y: {center[1]:.3f} z: {center[2]:.3f} }} '
                f'orientation {{ {orientation} }} }}',
                'material { diffuse { r: 1.0 g: 0.78 b: 0.02 a: 1.0 } '
                'ambient { r: 0.55 g: 0.3 b: 0.0 a: 1.0 } '
                'emissive { r: 0.4 g: 0.18 b: 0.0 a: 1.0 } }',
                'visibility: ALL',
            ]))

        with self._publish_lock:
            try:
                self._send_marker('action: DELETE_ALL ns: "drone_demo_route" id: 0')
                failed = 0
                for marker in segment_markers:
                    result = self._send_marker(marker)
                    if result.returncode != 0:
                        failed += 1

                for index, point in enumerate(route_points[2:-2], start=100):
                    waypoint_marker = ' '.join([
                        'action: ADD_MODIFY',
                        'ns: "drone_demo_route"',
                        f'id: {index}',
                        'type: SPHERE',
                        'scale { x: 1.4 y: 1.4 z: 1.4 }',
                        'pose { position { '
                        f'x: {point[0]:.3f} y: {point[1]:.3f} z: {point[2]:.3f} '
                        '} orientation { w: 1 } }',
                        'material { diffuse { r: 1.0 g: 0.12 b: 0.02 a: 1.0 } '
                        'ambient { r: 0.5 g: 0.02 b: 0.0 a: 1.0 } '
                        'emissive { r: 0.25 g: 0.0 b: 0.0 a: 1.0 } }',
                        'visibility: ALL',
                    ])
                    self._send_marker(waypoint_marker)
                if failed:
                    self.get_logger().warning(
                        f'Unable to render {failed} of {len(segment_markers)} route segments'
                    )
                self.get_logger().info(
                    f'Rendered mission route with {len(route_points)} points '
                    f'and {len(segment_markers)} visible segments'
                )
            except (OSError, subprocess.SubprocessError) as error:
                self.get_logger().warning(f'Unable to render mission route: {error}')

    @staticmethod
    def _send_marker(marker):
        return subprocess.run(
                    [
                        'gz', 'service', '-s', '/marker',
                        '--reqtype', 'gz.msgs.Marker',
                        '--reptype', 'gz.msgs.Empty',
                        '--timeout', '1000', '--req', marker,
                    ],
                    check=False,
                    env=os.environ.copy(),
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE,
                    text=True,
                    timeout=10,
                )


def main(args=None):
    rclpy.init(args=args)
    node = RouteVisualizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

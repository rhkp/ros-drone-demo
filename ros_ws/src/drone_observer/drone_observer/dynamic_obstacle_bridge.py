import math
import os

import rclpy
import yaml
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class DynamicObstacleBridge(Node):
    """Project global PoseArray obstacles into a Nav2-compatible LaserScan."""

    def __init__(self):
        super().__init__('dynamic_obstacle_bridge')
        scenario_file = self.declare_parameter(
            'scenario_file', os.environ.get('DRONE_SCENARIO_FILE', '/opt/drone-demo/config/scenarios.yaml')
        ).value
        with open(scenario_file, encoding='utf-8') as stream:
            config = yaml.safe_load(stream)
        dynamic = config.get('navigation', {}).get('dynamic_obstacles', {})
        self.topic = str(dynamic.get('topic', '/drone/dynamic_obstacles'))
        self.half_extent = float(dynamic.get('half_extent', 1.5))
        self.obstacles = []
        self.pose = None
        self.scan_pub = self.create_publisher(LaserScan, '/drone/dynamic_obstacles_scan', 10)
        self.create_subscription(PoseArray, self.topic, self.on_obstacles, 10)
        self.create_subscription(Odometry, '/drone/nav_odom', self.on_odom, 10)
        self.create_timer(0.1, self.publish_scan)

    def on_obstacles(self, message):
        self.obstacles = [(pose.position.x, pose.position.y) for pose in message.poses]

    def on_odom(self, message):
        self.pose = (message.pose.pose.position.x, message.pose.pose.position.y)

    def publish_scan(self):
        if self.pose is None:
            return
        count = 360
        increment = 2.0 * math.pi / count
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_link'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi - increment
        scan.angle_increment = increment
        scan.range_min = 0.1
        scan.range_max = 100.0
        scan.scan_time = 0.1
        scan.time_increment = scan.scan_time / count
        scan.ranges = [float('inf')] * count
        for obstacle_x, obstacle_y in self.obstacles:
            dx, dy = obstacle_x - self.pose[0], obstacle_y - self.pose[1]
            distance = math.hypot(dx, dy)
            if distance < scan.range_min or distance > scan.range_max:
                continue
            angle = math.atan2(dy, dx)
            span = math.atan2(self.half_extent, max(distance - self.half_extent, 0.1))
            for offset in (-span, 0.0, span):
                index = int(round((angle + math.pi + offset) / increment)) % count
                scan.ranges[index] = max(scan.range_min, distance - self.half_extent)
        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

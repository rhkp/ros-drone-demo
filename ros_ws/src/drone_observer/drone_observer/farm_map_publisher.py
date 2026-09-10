import os

import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class FarmMapPublisher(Node):
    """Publish a small transient-local map for Nav2's static costmap layer."""

    def __init__(self):
        super().__init__('farm_map_publisher')
        scenario_file = self.declare_parameter(
            'scenario_file', os.environ.get('DRONE_SCENARIO_FILE', '/opt/drone-demo/config/scenarios.yaml')
        ).value
        self.resolution = float(self.declare_parameter('resolution', 0.5).value)
        with open(scenario_file, encoding='utf-8') as stream:
            config = yaml.safe_load(stream)
        navigation = config.get('navigation', {})
        geofence = navigation.get('geofence', {})
        self.min_x = float(geofence.get('min_x', -35.0))
        self.max_x = float(geofence.get('max_x', 35.0))
        self.min_y = float(geofence.get('min_y', -25.0))
        self.max_y = float(geofence.get('max_y', 25.0))
        self.zones = navigation.get('no_fly_zones', [])
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(OccupancyGrid, '/map', qos)
        self.create_timer(1.0, self.publish_map)
        self.publish_map()

    def publish_map(self):
        width = int(round((self.max_x - self.min_x) / self.resolution))
        height = int(round((self.max_y - self.min_y) / self.resolution))
        message = OccupancyGrid()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.info.resolution = self.resolution
        message.info.width = width
        message.info.height = height
        message.info.origin.position.x = self.min_x
        message.info.origin.position.y = self.min_y
        message.info.origin.orientation.w = 1.0
        message.data = [0] * (width * height)
        for zone in self.zones:
            # Publish the physical restricted area. Nav2's inflation layer
            # supplies vehicle clearance; the mission planner applies the
            # configured route margin when it creates detour sub-goals.
            min_x = float(zone['min_x'])
            max_x = float(zone['max_x'])
            min_y = float(zone['min_y'])
            max_y = float(zone['max_y'])
            for row in range(height):
                center_y = self.min_y + (row + 0.5) * self.resolution
                if not min_y <= center_y <= max_y:
                    continue
                for column in range(width):
                    center_x = self.min_x + (column + 0.5) * self.resolution
                    if min_x <= center_x <= max_x:
                        message.data[row * width + column] = 100
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = FarmMapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

import os

import rclpy
import yaml
from drone_observer_msgs.msg import TargetTruth, TargetTruthArray
from geometry_msgs.msg import Pose
from rclpy.node import Node


class TruthPublisher(Node):
    def __init__(self):
        super().__init__('truth_publisher')
        path = self.declare_parameter('scenario_file', os.environ.get('DRONE_SCENARIO_FILE', '/opt/drone-demo/config/scenarios.yaml')).value
        with open(path, encoding='utf-8') as stream:
            self.config = yaml.safe_load(stream)
        self.pub = self.create_publisher(TargetTruthArray, '/drone/target_truth', 10)
        self.create_timer(1.0, self.publish_truth)

    def publish_truth(self):
        message = TargetTruthArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'farm_map'
        for target in self.config.get('targets', []):
            item = TargetTruth()
            item.id = target['id']
            item.target_type = target['type']
            item.pose = Pose()
            item.pose.position.x = float(target['x'])
            item.pose.position.y = float(target['y'])
            item.pose.position.z = float(target.get('z', 0.0))
            item.pose.orientation.w = 1.0
            message.targets.append(item)
        self.pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = TruthPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

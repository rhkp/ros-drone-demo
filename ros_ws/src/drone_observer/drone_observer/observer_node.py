import json
import math
import os
import time
from pathlib import Path

import rclpy
import yaml
from drone_observer_msgs.action import SurveyMission
from drone_observer_msgs.msg import TargetDetection, TargetDetectionArray, TargetTruthArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image

from .image_utils import synthetic_rgb, write_rgb_png


class ObserverNode(Node):
    def __init__(self):
        super().__init__('observer_node')
        scenario_file = self.declare_parameter('scenario_file', os.environ.get('DRONE_SCENARIO_FILE', '/opt/drone-demo/config/scenarios.yaml')).value
        self.default_scenario = self.declare_parameter('scenario', os.environ.get('DRONE_SCENARIO', 'all')).value
        self.artifact_dir = Path(self.declare_parameter('artifact_dir', os.environ.get('DRONE_ARTIFACT_DIR', '/tmp/drone-artifacts')).value)
        with open(scenario_file, encoding='utf-8') as stream:
            self.config = yaml.safe_load(stream)
        self.truth = {}
        self.latest_image = None
        self.callback_group = ReentrantCallbackGroup()
        self.setpoint_pub = self.create_publisher(PoseStamped, '/drone/setpoint', 10)
        self.detection_pub = self.create_publisher(TargetDetectionArray, '/drone/detections', 10)
        self.create_subscription(TargetTruthArray, '/drone/target_truth', self.on_truth, 10, callback_group=self.callback_group)
        self.create_subscription(Image, '/drone/camera/image_raw', self.on_image, 10, callback_group=self.callback_group)
        self.create_subscription(Odometry, '/drone/odom', self.on_odom, 10, callback_group=self.callback_group)
        self.server = ActionServer(self, SurveyMission, '/drone/survey', self.execute,
                                   goal_callback=self.accept_goal, callback_group=self.callback_group)
        self.get_logger().info(f'Farm observer ready; default scenario: {self.default_scenario}')

    def accept_goal(self, goal_request):
        scenario = goal_request.scenario or self.default_scenario
        if scenario not in self.config.get('scenarios', {}):
            self.get_logger().warning(f'Rejecting unknown scenario: {scenario}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def on_truth(self, message):
        self.truth = {target.id: target for target in message.targets}

    def on_image(self, message):
        self.latest_image = message

    def on_odom(self, message):
        position = message.pose.pose.position
        self._last_pose = (position.x, position.y, position.z)

    def publish_setpoint(self, x, y, z):
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'farm_map'
        message.pose.position.x, message.pose.position.y, message.pose.position.z = x, y, z
        message.pose.orientation.w = 1.0
        self.setpoint_pub.publish(message)

    def execute(self, goal_handle):
        request = goal_handle.request
        scenario_name = request.scenario or self.default_scenario
        scenario = self.config['scenarios'][scenario_name]
        mission_id = request.mission_id or f'{scenario_name}-{int(time.time())}'
        self.artifact_dir.mkdir(parents=True, exist_ok=True)
        waypoints = scenario['waypoints']
        allowed_types = set(scenario['target_types'])
        detections = []
        for index, waypoint in enumerate(waypoints):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return SurveyMission.Result(success=False, detections=len(detections), message='Mission canceled')
            x, y, z = float(waypoint['x']), float(waypoint['y']), float(waypoint.get('z', 12.0))
            self.publish_setpoint(x, y, z)
            self._wait_for_position(x, y, z)
            visible = self._targets_near(x, y, allowed_types)
            for target in visible:
                evidence = self._capture(mission_id, target.id)
                detection = TargetDetection()
                detection.header.stamp = self.get_clock().now().to_msg()
                detection.header.frame_id = 'farm_map'
                detection.mission_id, detection.target_id = mission_id, target.id
                detection.target_type = target.target_type
                detection.confidence = 0.98
                detection.pose = target.pose
                detection.image_path = str(evidence)
                detections.append(detection)
            self._publish_detections(detections)
            feedback = SurveyMission.Feedback()
            feedback.phase, feedback.detections = f'waypoint {index + 1}/{len(waypoints)}', len(detections)
            feedback.target_id = visible[0].id if visible else ''
            goal_handle.publish_feedback(feedback)
        report = self._write_report(mission_id, scenario_name, detections)
        goal_handle.succeed()
        result = SurveyMission.Result()
        result.success, result.detections, result.report_path = True, len(detections), str(report)
        result.message = f'Farm survey complete: {scenario_name}'
        return result

    def _wait_for_position(self, x, y, z):
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline and rclpy.ok():
            self.publish_setpoint(x, y, z)
            pose = getattr(self, '_last_pose', None)
            if pose and math.dist(pose, (x, y, z)) < 0.5:
                return
            time.sleep(0.1)

    def _targets_near(self, x, y, allowed_types):
        return [target for target in self.truth.values()
                if target.target_type in allowed_types and math.hypot(target.pose.position.x - x, target.pose.position.y - y) < 8.0]

    def _capture(self, mission_id, target_id):
        path = self.artifact_dir / mission_id / f'{target_id}.png'
        path.parent.mkdir(parents=True, exist_ok=True)
        if self.latest_image and self.latest_image.encoding == 'rgb8' and self.latest_image.step == self.latest_image.width * 3:
            write_rgb_png(path, self.latest_image.width, self.latest_image.height, bytes(self.latest_image.data))
        else:
            write_rgb_png(path, 320, 240, synthetic_rgb(320, 240))
        return path

    def _publish_detections(self, detections):
        message = TargetDetectionArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'farm_map'
        message.detections = detections
        self.detection_pub.publish(message)

    def _write_report(self, mission_id, scenario_name, detections):
        path = self.artifact_dir / mission_id / 'report.json'
        path.parent.mkdir(parents=True, exist_ok=True)
        report = {
            'mission_id': mission_id,
            'scenario': scenario_name,
            'detections': [
                {'target_id': item.target_id, 'target_type': item.target_type,
                 'confidence': item.confidence, 'image_path': item.image_path}
                for item in detections
            ],
        }
        path.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
        return path


def main(args=None):
    rclpy.init(args=args)
    node = ObserverNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.spin()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

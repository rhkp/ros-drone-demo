"""Offline-only evaluator for camera detector predictions.

This node is deliberately separate from runtime inference. It may subscribe to
simulator truth for scoring, but it never publishes mission detections or feeds
truth back into the detector.
"""

import json
import os
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from drone_observer_msgs.msg import TargetDetectionArray, TargetTruthArray

from .label_projection import project_target_box


def iou(left, right):
    x1, y1 = max(left[0], right[0]), max(left[1], right[1])
    x2, y2 = min(left[2], right[2]), min(left[3], right[3])
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    left_area = max(0.0, left[2] - left[0]) * max(0.0, left[3] - left[1])
    right_area = max(0.0, right[2] - right[0]) * max(0.0, right[3] - right[1])
    union = left_area + right_area - intersection
    return intersection / union if union else 0.0


class ShadowEvaluator(Node):
    def __init__(self):
        super().__init__('shadow_evaluator')
        scenario_file = str(self.declare_parameter(
            'scenario_file', os.environ.get('DRONE_SCENARIO_FILE', '/opt/drone-demo/config/scenarios.yaml')
        ).value)
        self.output_path = Path(self.declare_parameter(
            'output_path', os.environ.get(
                'DRONE_SHADOW_REPORT', '/data/perception/evaluations/shadow-v6/report.json'
            )
        ).value)
        self.duration_sec = float(self.declare_parameter(
            'duration_sec', float(os.environ.get('DRONE_SHADOW_DURATION_SEC', '240'))
        ).value)
        self.iou_threshold = float(self.declare_parameter(
            'iou_threshold', float(os.environ.get('DRONE_SHADOW_IOU_THRESHOLD', '0.5'))
        ).value)
        self.model_version = str(self.declare_parameter(
            'model_version', os.environ.get('DRONE_SHADOW_MODEL_VERSION', 'v6')
        ).value)
        with open(scenario_file, encoding='utf-8') as stream:
            self.config = yaml.safe_load(stream)
        dataset = self.config.get('dataset', {})
        self.class_names = list(dataset.get('class_order', []))
        self.targets = {item['id']: item for item in self.config.get('targets', [])}
        camera = dataset.get('camera', {})
        self.camera_config = {
            'offset_z': float(camera.get('offset_z', 0.36)),
            'image_x_axis': camera.get('image_x_axis', 'x'),
            'image_x_sign': float(camera.get('image_x_sign', 1)),
            'image_y_axis': camera.get('image_y_axis', 'y'),
            'image_y_sign': float(camera.get('image_y_sign', 1)),
        }
        self.camera = None
        self.pose = None
        self.truth = {}
        self.started = time.monotonic()
        self.finished = False
        self.mission_active = False
        self.frames = 0
        self.predictions = 0
        self.latencies = []
        self.stats = {name: {'tp': 0, 'fp': 0, 'fn': 0} for name in self.class_names}
        self.create_subscription(CameraInfo, '/drone/camera/camera_info', self.on_camera, 10)
        self.create_subscription(Odometry, '/drone/odom', self.on_odom, 10)
        self.create_subscription(TargetTruthArray, '/drone/target_truth', self.on_truth, 10)
        self.create_subscription(TargetDetectionArray, '/drone/camera_detections', self.on_predictions, 10)
        self.create_subscription(String, '/drone/mission_state', self.on_mission_state, 10)
        self.create_timer(1.0, self.on_timer)
        self.get_logger().info(f'Shadow evaluator ready: output={self.output_path}, duration={self.duration_sec}s')

    def on_camera(self, message):
        width, height = int(message.width), int(message.height)
        k = list(message.k)
        fx = float(k[0]) if k and k[0] else width / (2.0 * 0.5773502692)
        fy = float(k[4]) if len(k) > 4 and k[4] else fx
        cx = float(k[2]) if len(k) > 2 and k[2] else width / 2.0
        cy = float(k[5]) if len(k) > 5 and k[5] else height / 2.0
        self.camera = {**self.camera_config, 'width': width, 'height': height,
                       'fx': fx, 'fy': fy, 'cx': cx, 'cy': cy}

    def on_odom(self, message):
        position = message.pose.pose.position
        self.pose = (position.x, position.y, position.z)

    def on_truth(self, message):
        self.truth = {target.id: target for target in message.targets}

    def on_mission_state(self, message):
        self.mission_active = message.data in {
            'TAKEOFF', 'TRANSIT', 'INSPECT', 'RETURN', 'LAND'
        }

    def truth_boxes(self):
        if self.camera is None or self.pose is None:
            return []
        boxes = []
        for target_id, truth in self.truth.items():
            config = self.targets.get(target_id)
            if not config:
                continue
            center = {'x': truth.pose.position.x, 'y': truth.pose.position.y, 'z': truth.pose.position.z}
            box = project_target_box(center, config['dimensions'], self.pose, self.camera)
            if box:
                boxes.append((config['type'], [box['x_min'], box['y_min'], box['x_max'], box['y_max']]))
        return boxes

    def on_predictions(self, message):
        if not self.mission_active:
            return
        truths = self.truth_boxes()
        predictions = []
        for detection in message.detections:
            box = [detection.bbox_x_min, detection.bbox_y_min, detection.bbox_x_max, detection.bbox_y_max]
            if box[2] > box[0] and box[3] > box[1]:
                predictions.append((detection.target_type, box, float(detection.confidence)))
        matched = set()
        for name, box, _score in sorted(predictions, key=lambda item: item[2], reverse=True):
            self.predictions += 1
            candidates = [(index, iou(box, truth_box)) for index, (truth_name, truth_box) in enumerate(truths)
                          if truth_name == name and index not in matched]
            best = max(candidates, key=lambda item: item[1], default=(-1, 0.0))
            if name not in self.stats:
                self.stats[name] = {'tp': 0, 'fp': 0, 'fn': 0}
            if best[1] >= self.iou_threshold:
                matched.add(best[0])
                self.stats[name]['tp'] += 1
            else:
                self.stats[name]['fp'] += 1
        for index, (name, _box) in enumerate(truths):
            if index not in matched and name in self.stats:
                self.stats[name]['fn'] += 1
        self.frames += 1
        self.latencies.append(float(message.inference_latency_ms))

    def on_timer(self):
        if not self.finished and time.monotonic() - self.started >= self.duration_sec:
            self.write_report()
            self.finished = True
            rclpy.shutdown()

    def write_report(self):
        metrics = {}
        precisions, recalls = [], []
        for name in self.class_names:
            values = self.stats[name]
            precision = values['tp'] / max(1, values['tp'] + values['fp'])
            recall = values['tp'] / max(1, values['tp'] + values['fn'])
            metrics[name] = {**values, 'precision': precision, 'recall': recall}
            precisions.append(precision)
            recalls.append(recall)
        report = {
            'schema_version': 'shadow-evaluation-report-1',
            'model_version': self.model_version,
            'source_topic': '/drone/camera_detections',
            'truth_topic': '/drone/target_truth',
            'truth_usage': 'offline_scoring_only',
            'iou_threshold': self.iou_threshold,
            'frames': self.frames,
            'predictions': self.predictions,
            'macro_precision': sum(precisions) / max(1, len(precisions)),
            'macro_recall': sum(recalls) / max(1, len(recalls)),
            'inference_latency_ms': sum(self.latencies) / max(1, len(self.latencies)),
            'per_class': metrics,
        }
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_path.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
        self.get_logger().info(json.dumps({'event': 'shadow_evaluation_complete', **report}, indent=2))


def main(args=None):
    rclpy.init(args=args)
    node = ShadowEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.finished:
            node.write_report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

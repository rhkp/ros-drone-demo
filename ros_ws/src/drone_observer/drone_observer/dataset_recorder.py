"""Offline synthetic dataset recorder for the farm drone camera.

This node deliberately subscribes to target truth because it is a data
generation tool. The generated dataset records that fact in its manifest and
metadata. The runtime observer/detector must not use this node or topic.
"""

import json
import os
import time
from pathlib import Path

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from drone_observer_msgs.msg import TargetTruthArray

from .image_utils import image_to_rgb, write_rgb_png
from .label_projection import normalize_box, project_target_box


class DatasetRecorder(Node):
    def __init__(self):
        super().__init__('dataset_recorder')
        scenario_file = self.declare_parameter(
            'scenario_file', os.environ.get('DRONE_SCENARIO_FILE', '/opt/drone-demo/config/scenarios.yaml')
        ).value
        self.scenario_name = self.declare_parameter(
            'scenario', os.environ.get('DRONE_SCENARIO', 'all')
        ).value
        self.output_dir = Path(self.declare_parameter(
            'output_dir', os.environ.get('DRONE_DATASET_DIR', '/data/perception/datasets/v1')
        ).value)
        self.capture_interval = float(self.declare_parameter(
            'capture_interval_sec', float(os.environ.get('DRONE_CAPTURE_INTERVAL_SEC', '0.5'))
        ).value)
        self.max_frames = int(self.declare_parameter(
            'max_frames', int(os.environ.get('DRONE_MAX_FRAMES', '0'))
        ).value)
        self.seed = str(self.declare_parameter(
            'seed', os.environ.get('DRONE_DATASET_SEED', 'default')
        ).value)
        with open(scenario_file, encoding='utf-8') as stream:
            self.config = yaml.safe_load(stream)
        scenarios = self.config.get('scenarios', {})
        if self.scenario_name not in scenarios:
            raise ValueError(f'Unknown scenario: {self.scenario_name}')
        dataset_config = self.config.get('dataset', {})
        configured_classes = dataset_config.get('class_order', [])
        self.class_order = list(configured_classes) or list(dict.fromkeys(
            target_type for item in self.config.get('targets', [])
            for target_type in [item['type']]
        ))
        self.class_ids = {name: index for index, name in enumerate(self.class_order)}
        self.targets = {item['id']: item for item in self.config.get('targets', [])}
        self.allowed_types = set(scenarios[self.scenario_name].get('target_types', self.class_order))
        camera_config = dataset_config.get('camera', {})
        self.camera_offset_z = float(camera_config.get('offset_z', 0.36))
        self.image_x_axis = str(camera_config.get('image_x_axis', 'x'))
        self.image_x_sign = float(camera_config.get('image_x_sign', 1))
        self.image_y_axis = str(camera_config.get('image_y_axis', 'y'))
        self.image_y_sign = float(camera_config.get('image_y_sign', 1))

        self.latest_camera = None
        self.latest_image = None
        self.latest_pose = None
        self.latest_truth = {}
        self.last_capture_time = 0.0
        self.frame_count = 0
        self.started_at = time.time()
        self.session_id = f'{self.scenario_name}-{time.strftime("%Y%m%d-%H%M%S", time.gmtime(self.started_at))}'
        self.output_dir.mkdir(parents=True, exist_ok=True)
        (self.output_dir / 'images').mkdir(exist_ok=True)
        (self.output_dir / 'labels').mkdir(exist_ok=True)
        (self.output_dir / 'metadata').mkdir(exist_ok=True)
        self.manifest_path = self.output_dir / 'dataset_manifest.json'
        self._write_manifest()

        self.create_subscription(Image, '/drone/camera/image_raw', self.on_image, 10)
        self.create_subscription(CameraInfo, '/drone/camera/camera_info', self.on_camera_info, 10)
        self.create_subscription(Odometry, '/drone/odom', self.on_odom, 10)
        self.create_subscription(TargetTruthArray, '/drone/target_truth', self.on_truth, 10)
        self.get_logger().info(
            f'Dataset recorder ready: scenario={self.scenario_name}, output={self.output_dir}, '
            f'classes={self.class_order}, truth_topic=offline-only'
        )

    def on_camera_info(self, message):
        self.latest_camera = message

    def on_odom(self, message):
        position = message.pose.pose.position
        self.latest_pose = (position.x, position.y, position.z)

    def on_truth(self, message):
        self.latest_truth = {target.id: target for target in message.targets}

    def on_image(self, message):
        self.latest_image = message
        now = time.monotonic()
        if now - self.last_capture_time < self.capture_interval:
            return
        if self.max_frames and self.frame_count >= self.max_frames:
            return
        if self.latest_camera is None or self.latest_pose is None or not self.latest_truth:
            return
        try:
            self.capture_frame(message)
        except Exception as error:  # keep the recorder alive for a bad frame
            self.get_logger().error(f'Unable to capture dataset frame: {error}')
            return
        self.last_capture_time = now
        if self.max_frames and self.frame_count >= self.max_frames:
            self.get_logger().info(f'Reached max_frames={self.max_frames}; recorder remains idle')

    def camera_model(self, message):
        k = list(message.k)
        width, height = int(message.width), int(message.height)
        fx = float(k[0]) if len(k) > 0 and k[0] else width / (2.0 * 0.5773502692)
        fy = float(k[4]) if len(k) > 4 and k[4] else fx
        cx = float(k[2]) if len(k) > 2 and k[2] else width / 2.0
        cy = float(k[5]) if len(k) > 5 and k[5] else height / 2.0
        return {
            'width': width, 'height': height, 'fx': fx, 'fy': fy,
            'cx': cx, 'cy': cy, 'frame_id': message.header.frame_id,
        }

    def capture_frame(self, image):
        self.frame_count += 1
        frame_name = f'frame-{self.frame_count:06d}'
        image_path = self.output_dir / 'images' / f'{frame_name}.png'
        label_json_path = self.output_dir / 'labels' / f'{frame_name}.json'
        label_txt_path = self.output_dir / 'labels' / f'{frame_name}.txt'
        metadata_path = self.output_dir / 'metadata' / f'{frame_name}.json'
        model = self.camera_model(self.latest_camera)
        write_rgb_png(image_path, int(image.width), int(image.height), image_to_rgb(image))

        annotations = []
        yolo_lines = []
        for target_id, truth in sorted(self.latest_truth.items()):
            target = self.targets.get(target_id)
            if not target or target['type'] not in self.allowed_types:
                continue
            dimensions = target.get('dimensions')
            if not dimensions:
                self.get_logger().warning(f'Skipping {target_id}: target dimensions are missing')
                continue
            target_mapping = {'x': truth.pose.position.x, 'y': truth.pose.position.y,
                              'z': truth.pose.position.z}
            box = project_target_box(
            target_mapping, dimensions, self.latest_pose,
                {**model, 'offset_z': self.camera_offset_z,
                 'image_x_axis': self.image_x_axis,
                 'image_x_sign': self.image_x_sign,
                 'image_y_axis': self.image_y_axis,
                 'image_y_sign': self.image_y_sign},
                self.image_y_sign,
            )
            if box is None:
                continue
            normalized = normalize_box(box, image.width, image.height)
            annotation = {
                'target_id': target_id,
                'class_name': target['type'],
                'class_id': self.class_ids[target['type']],
                'dimensions_m': dimensions,
                'center_farm_map': target_mapping,
                'bbox_pixels': box,
                'bbox_normalized': normalized,
                'label_source': 'simulator_truth_offline',
            }
            annotations.append(annotation)
            yolo_lines.append(
                f"{annotation['class_id']} {normalized['center_x']:.6f} "
                f"{normalized['center_y']:.6f} {normalized['width']:.6f} "
                f"{normalized['height']:.6f}"
            )
        label_json_path.write_text(json.dumps({
            'schema_version': 'synthetic-detection-label-1',
            'image': str(image_path.relative_to(self.output_dir)),
            'image_width': int(image.width),
            'image_height': int(image.height),
            'annotations': annotations,
        }, indent=2) + '\n', encoding='utf-8')
        label_txt_path.write_text('\n'.join(yolo_lines) + ('\n' if yolo_lines else ''), encoding='utf-8')
        metadata_path.write_text(json.dumps({
            'schema_version': 'synthetic-frame-metadata-1',
            'session_id': self.session_id,
            'scenario': self.scenario_name,
            'seed': self.seed,
            'captured_at_unix': time.time(),
            'image_topic': '/drone/camera/image_raw',
            'camera_info_topic': '/drone/camera/camera_info',
            'odom_topic': '/drone/odom',
            'truth_topic': '/drone/target_truth',
            'truth_usage': 'offline_label_generation_only',
            'drone_pose_farm_map': self.latest_pose,
            'camera': model,
            'image': str(image_path.relative_to(self.output_dir)),
            'labels': str(label_json_path.relative_to(self.output_dir)),
            'annotation_count': len(annotations),
        }, indent=2) + '\n', encoding='utf-8')
        self._write_manifest()
        self.get_logger().info(
            f'Captured {frame_name}: {len(annotations)} label(s), '
            f'{image.width}x{image.height}'
        )

    def _write_manifest(self):
        class_counts = {name: 0 for name in self.class_order}
        for label_path in sorted((self.output_dir / 'labels').glob('*.json')):
            payload = json.loads(label_path.read_text(encoding='utf-8'))
            for annotation in payload.get('annotations', []):
                class_counts[annotation['class_name']] += 1
        manifest = {
            'schema_version': 'synthetic-dataset-manifest-1',
            'dataset_version': self.output_dir.name,
            'session_id': self.session_id,
            'scenario': self.scenario_name,
            'seed': self.seed,
            'class_order': self.class_order,
            'class_ids': self.class_ids,
            'class_counts': class_counts,
            'frames': self.frame_count,
            'images_dir': 'images',
            'labels_dir': 'labels',
            'metadata_dir': 'metadata',
            'label_format': 'YOLO normalized txt plus JSON audit labels',
            'label_source': 'simulator_truth_offline',
            'camera_projection': {
                'mount': 'downward',
                'offset_z_m': self.camera_offset_z,
                'image_x_axis': self.image_x_axis,
                'image_x_sign': self.image_x_sign,
                'image_y_axis': self.image_y_axis,
                'image_y_sign': self.image_y_sign,
            },
            'split_policy': 'split by scene or seed, never adjacent frames',
        }
        self.manifest_path.write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')


def main(args=None):
    rclpy.init(args=args)
    node = DatasetRecorder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

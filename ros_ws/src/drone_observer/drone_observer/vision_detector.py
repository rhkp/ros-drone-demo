"""Camera-only Faster R-CNN detector for live perception.

This node deliberately subscribes only to the camera image topic and publishes
predictions to a separate topic consumed by the mission observer and the ML
pipeline. It runs the trained PyTorch checkpoint directly on the GPU when one
is available.
"""

import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from drone_observer_msgs.msg import TargetDetection, TargetDetectionArray

from .image_utils import image_to_rgb


class VisionDetector(Node):
    def __init__(self):
        super().__init__('vision_detector')
        model_path = self.declare_parameter(
            'model_path', os.environ.get('DRONE_MODEL_PATH', '/data/perception/models/v7/detector.pt')
        ).value
        class_names = self.declare_parameter(
            'class_names', os.environ.get(
                'DRONE_CLASS_NAMES',
                'well,storage_silo,cattle_shed,produce_barn,crop_field,greenhouse,water_tank',
            )
        ).value
        self.class_names = [item.strip() for item in str(class_names).split(',') if item.strip()]
        self.model_version = str(self.declare_parameter(
            'model_version', os.environ.get('DRONE_MODEL_VERSION', 'v7')
        ).value)
        self.confidence_threshold = float(self.declare_parameter(
            'confidence_threshold', float(os.environ.get('DRONE_CONFIDENCE_THRESHOLD', '0.35'))
        ).value)
        self.output_topic = str(self.declare_parameter(
            'output_topic', os.environ.get('DRONE_DETECTION_TOPIC', '/drone/camera_detections')
        ).value)
        self.input_width = int(self.declare_parameter('input_width', 320).value)
        self.input_height = int(self.declare_parameter('input_height', 240).value)
        import torch
        from torchvision.models.detection import fasterrcnn_mobilenet_v3_large_320_fpn

        checkpoint = torch.load(model_path, map_location='cpu', weights_only=False)
        self.torch = torch
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.detector = fasterrcnn_mobilenet_v3_large_320_fpn(
            weights=None, weights_backbone=None, num_classes=len(self.class_names) + 1,
        )
        self.detector.load_state_dict(checkpoint['state_dict'])
        self.detector.to(self.device).eval()
        self.output = self.create_publisher(TargetDetectionArray, self.output_topic, 10)
        self.create_subscription(Image, '/drone/camera/image_raw', self.on_image, 10)
        self.frame_count = 0
        self.get_logger().info(
            f'Camera-only detector ready: model={model_path}, version={self.model_version}, '
            f'runtime=pytorch, device={self.device}, output={self.output_topic}, classes={self.class_names}'
        )

    def tensor_from_image(self, message):
        width, height = int(message.width), int(message.height)
        rgb = np.frombuffer(image_to_rgb(message), dtype=np.uint8).reshape(height, width, 3)
        if (width, height) != (self.input_width, self.input_height):
            x_indices = np.linspace(0, width - 1, self.input_width).round().astype(np.int64)
            y_indices = np.linspace(0, height - 1, self.input_height).round().astype(np.int64)
            rgb = rgb[y_indices][:, x_indices]
        tensor = np.transpose(rgb, (2, 0, 1)).astype(np.float32) / 255.0
        return tensor[None, ...], width, height

    def on_image(self, message):
        started = time.perf_counter()
        try:
            tensor, original_width, original_height = self.tensor_from_image(message)
            image_tensor = self.torch.from_numpy(tensor[0]).to(self.device)
            with self.torch.no_grad():
                prediction = self.detector([image_tensor])[0]
            boxes = prediction['boxes'].detach().cpu().numpy()
            labels = prediction['labels'].detach().cpu().numpy()
            scores = prediction['scores'].detach().cpu().numpy()
        except Exception as error:
            self.get_logger().error(f'Unable to infer camera frame: {error}')
            failed = TargetDetectionArray()
            failed.header = message.header
            failed.model_version = self.model_version
            failed.inference_latency_ms = (time.perf_counter() - started) * 1000.0
            failed.inference_ok = False
            failed.inference_error = str(error)[:512]
            self.output.publish(failed)
            return
        scale_x = original_width / float(self.input_width)
        scale_y = original_height / float(self.input_height)
        detections = []
        self.frame_count += 1
        for box, label, score in zip(boxes, labels, scores):
            confidence = float(score)
            class_id = int(label)
            if confidence < self.confidence_threshold or not 1 <= class_id <= len(self.class_names):
                continue
            detection = TargetDetection()
            detection.header = message.header
            detection.target_id = f'camera-frame-{self.frame_count:06d}'
            detection.target_type = self.class_names[class_id - 1]
            detection.confidence = confidence
            detection.image_path = ''
            detection.bbox_x_min = float(box[0]) * scale_x
            detection.bbox_y_min = float(box[1]) * scale_y
            detection.bbox_x_max = float(box[2]) * scale_x
            detection.bbox_y_max = float(box[3]) * scale_y
            detection.model_version = self.model_version
            detection.pose.orientation.w = 1.0
            detections.append(detection)
        result = TargetDetectionArray()
        result.header = message.header
        result.detections = detections
        result.inference_latency_ms = (time.perf_counter() - started) * 1000.0
        result.model_version = self.model_version
        result.inference_ok = True
        self.output.publish(result)
        latency_ms = result.inference_latency_ms
        self.get_logger().debug(f'frame={self.frame_count} detections={len(detections)} latency_ms={latency_ms:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = VisionDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

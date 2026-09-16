import json
import os
import threading
import time
from pathlib import Path

import rclpy
import yaml
from drone_observer_msgs.action import SurveyMission
from drone_observer_msgs.msg import TargetDetection, TargetDetectionArray
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as NavPath
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String

from .image_utils import image_to_rgb, write_rgb_png
from .navigation import Geofence, NavigationCanceled, ObstacleZone
from .nav2_navigation import Nav2Navigator


class MissionCanceled(Exception):
    pass


class ObserverNode(Node):
    def __init__(self):
        super().__init__('observer_node')
        scenario_file = self.declare_parameter('scenario_file', os.environ.get('DRONE_SCENARIO_FILE', '/opt/drone-demo/config/scenarios.yaml')).value
        self.default_scenario = self.declare_parameter('scenario', os.environ.get('DRONE_SCENARIO', 'all')).value
        self.artifact_dir = Path(self.declare_parameter('artifact_dir', os.environ.get('DRONE_ARTIFACT_DIR', '/tmp/drone-artifacts')).value)
        with open(scenario_file, encoding='utf-8') as stream:
            self.config = yaml.safe_load(stream)
        navigation_config = self.config.get('navigation', {})
        geofence_config = navigation_config.get('geofence', {})
        obstacles = [ObstacleZone.from_mapping(item) for item in navigation_config.get('no_fly_zones', [])]
        dynamic_config = navigation_config.get('dynamic_obstacles', {})
        self.dynamic_obstacles_enabled = bool(dynamic_config.get('enabled', True))
        self.dynamic_obstacle_topic = str(dynamic_config.get('topic', '/drone/dynamic_obstacles'))
        self.dynamic_obstacle_half_extent = float(dynamic_config.get('half_extent', 1.5))
        self.dynamic_obstacle_min_z = float(dynamic_config.get('min_z', 0.0))
        self.dynamic_obstacle_max_z = float(dynamic_config.get('max_z', 20.0))
        self.dynamic_obstacle_margin = float(dynamic_config.get('margin', 1.0))
        self.dynamic_obstacle_ttl = float(dynamic_config.get('ttl_sec', 5.0))
        self._dynamic_obstacle_received_at = None
        self._dynamic_obstacle_count = 0
        self.navigation_progress_pub = self.create_publisher(Float32, '/drone/navigation_progress', 10)
        self.navigator = Nav2Navigator(
            self,
            self.publish_setpoint,
            lambda: self._last_pose,
            self._publish_navigation_progress,
            Geofence.from_mapping(geofence_config),
            obstacles,
        )
        self.home_x = self.declare_parameter('home_x', float(os.environ.get('DRONE_HOME_X', '0.0'))).value
        self.home_y = self.declare_parameter('home_y', float(os.environ.get('DRONE_HOME_Y', '0.0'))).value
        self.takeoff_altitude = self.declare_parameter(
            'takeoff_altitude', float(os.environ.get('DRONE_TAKEOFF_ALTITUDE', '12.0'))
        ).value
        self.landing_altitude = self.declare_parameter(
            'landing_altitude', float(os.environ.get('DRONE_LANDING_ALTITUDE', '0.6'))
        ).value
        self.camera_detection_topic = str(self.declare_parameter(
            'camera_detection_topic',
            os.environ.get('DRONE_DETECTION_TOPIC', '/drone/camera_detections'),
        ).value)
        self.camera_observation_window_sec = float(self.declare_parameter(
            'camera_observation_window_sec',
            float(os.environ.get('DRONE_CAMERA_OBSERVATION_WINDOW_SEC', '2.0')),
        ).value)
        self.camera_confidence_threshold = float(self.declare_parameter(
            'camera_confidence_threshold',
            float(os.environ.get('DRONE_MISSION_CONFIDENCE_THRESHOLD', '0.35')),
        ).value)
        self._latest_camera_detections = None
        self._camera_detection_sequence = 0
        self._camera_detection_condition = threading.Condition()
        self.latest_image = None
        self._last_pose = None
        self._mission_lock = threading.Lock()
        self._mission_active = False
        self._state_history = []
        self.current_state = 'IDLE'
        self.callback_group = ReentrantCallbackGroup()
        self.setpoint_pub = self.create_publisher(PoseStamped, '/drone/setpoint', 10)
        path_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(NavPath, '/drone/mission_path', path_qos)
        self.detection_pub = self.create_publisher(TargetDetectionArray, '/drone/detections', 10)
        self.state_pub = self.create_publisher(String, '/drone/mission_state', 10)
        self.create_subscription(
            TargetDetectionArray,
            self.camera_detection_topic,
            self.on_camera_detections,
            10,
            callback_group=self.callback_group,
        )
        self.create_subscription(Image, '/drone/camera/image_raw', self.on_image, 10, callback_group=self.callback_group)
        self.create_subscription(Odometry, '/drone/odom', self.on_odom, 10, callback_group=self.callback_group)
        if self.dynamic_obstacles_enabled:
            self.create_subscription(
                PoseArray, self.dynamic_obstacle_topic, self.on_dynamic_obstacles, 10,
                callback_group=self.callback_group,
            )
            self.create_timer(1.0, self._expire_dynamic_obstacles, callback_group=self.callback_group)
        self.server = ActionServer(self, SurveyMission, '/drone/survey', self.execute,
                                   goal_callback=self.accept_goal, cancel_callback=self.accept_cancel,
                                   callback_group=self.callback_group)
        self.get_logger().info(
            f'Farm observer ready; default scenario: {self.default_scenario}; '
            f'camera perception topic: {self.camera_detection_topic}'
        )

    def accept_goal(self, goal_request):
        scenario = goal_request.scenario or self.default_scenario
        if scenario not in self.config.get('scenarios', {}):
            self.get_logger().warning(f'Rejecting unknown scenario: {scenario}')
            return GoalResponse.REJECT
        with self._mission_lock:
            if self._mission_active:
                self.get_logger().warning('Rejecting mission because another mission is active')
                return GoalResponse.REJECT
            self._mission_active = True
        return GoalResponse.ACCEPT

    def accept_cancel(self, _goal_handle):
        self.get_logger().info('Accepting mission cancellation request')
        return CancelResponse.ACCEPT

    def on_camera_detections(self, message):
        with self._camera_detection_condition:
            self._latest_camera_detections = message
            self._camera_detection_sequence += 1
            self._camera_detection_condition.notify_all()

    def on_image(self, message):
        self.latest_image = message

    def on_odom(self, message):
        position = message.pose.pose.position
        self._last_pose = (position.x, position.y, position.z)

    def on_dynamic_obstacles(self, message):
        half_extent = self.dynamic_obstacle_half_extent
        obstacles = []
        for index, pose in enumerate(message.poses):
            x, y = pose.position.x, pose.position.y
            obstacles.append(ObstacleZone(
                f'dynamic_{index}', x - half_extent, x + half_extent,
                y - half_extent, y + half_extent,
                self.dynamic_obstacle_min_z, self.dynamic_obstacle_max_z,
                self.dynamic_obstacle_margin,
            ))
        self.navigator.set_dynamic_obstacles(obstacles)
        self._dynamic_obstacle_received_at = time.monotonic()
        if len(obstacles) != self._dynamic_obstacle_count:
            self._dynamic_obstacle_count = len(obstacles)
            self.get_logger().info(
                f'Runtime obstacle feed updated: {self._dynamic_obstacle_count} obstacle(s)'
            )

    def _expire_dynamic_obstacles(self):
        if (self._dynamic_obstacle_received_at is None or
                time.monotonic() - self._dynamic_obstacle_received_at <= self.dynamic_obstacle_ttl):
            return
        if self._dynamic_obstacle_count:
            self.navigator.set_dynamic_obstacles(())
            self._dynamic_obstacle_count = 0
            self.get_logger().info('Runtime obstacle feed expired; resuming with static zones only')
        self._dynamic_obstacle_received_at = None

    def publish_setpoint(self, x, y, z):
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'farm_map'
        message.pose.position.x, message.pose.position.y, message.pose.position.z = x, y, z
        message.pose.orientation.w = 1.0
        self.setpoint_pub.publish(message)

    def _set_state(self, state, goal_handle=None, detections=0, target_id=''):
        self.current_state = state
        self._state_history.append({'state': state, 'timestamp': time.time()})
        state_message = String()
        state_message.data = state
        self.state_pub.publish(state_message)
        self.get_logger().info(f'Mission state: {state}')
        if goal_handle is not None:
            feedback = SurveyMission.Feedback()
            feedback.phase = state
            feedback.target_id = target_id
            feedback.detections = detections
            goal_handle.publish_feedback(feedback)

    def execute(self, goal_handle):
        request = goal_handle.request
        scenario_name = request.scenario or self.default_scenario
        scenario = self.config['scenarios'][scenario_name]
        mission_id = request.mission_id or f'{scenario_name}-{int(time.time())}'
        self.artifact_dir.mkdir(parents=True, exist_ok=True)
        waypoints = scenario['waypoints']
        allowed_types = set(scenario['target_types'])
        route = []
        planned_route = []
        detections = []
        perception_observations = []
        self._state_history = []
        self.navigator.reset_replan_events()
        try:
            route = self._build_route(waypoints)
            planned_route = self.navigator.plan_route(route)
            self._publish_route(planned_route)
            self._set_state('TAKEOFF', goal_handle, len(detections))
            segment_count = len(route) - 1
            self._wait_for_position(
                self.home_x, self.home_y, self.takeoff_altitude, goal_handle,
                progress_start=0.0, progress_end=1.0 / segment_count,
            )

            for index, waypoint in enumerate(waypoints):
                x, y, z = float(waypoint['x']), float(waypoint['y']), float(waypoint.get('z', self.takeoff_altitude))
                self._set_state('TRANSIT', goal_handle, len(detections), f'waypoint {index + 1}/{len(waypoints)}')
                self._wait_for_position(
                    x, y, z, goal_handle,
                    progress_start=(index + 1) / segment_count,
                    progress_end=(index + 2) / segment_count,
                )
                self._set_state('INSPECT', goal_handle, len(detections))
                predictions, observation = self._observe_camera(allowed_types)
                observation['waypoint_index'] = index + 1
                observation['waypoint'] = {'x': x, 'y': y, 'z': z}
                perception_observations.append(observation)
                for prediction in predictions:
                    self._set_state('INSPECT', goal_handle, len(detections), prediction.target_type)
                    evidence = self._capture(
                        mission_id,
                        f'waypoint-{index + 1}-{prediction.target_type}',
                    )
                    detection = TargetDetection()
                    detection.header.stamp = prediction.header.stamp
                    detection.header.frame_id = 'farm_map'
                    detection.mission_id = mission_id
                    detection.target_id = prediction.target_id or f'camera-detection-{len(detections) + 1:04d}'
                    detection.target_type = prediction.target_type
                    detection.confidence = prediction.confidence
                    detection.pose.position.x = x
                    detection.pose.position.y = y
                    detection.pose.position.z = z
                    detection.pose.orientation.w = 1.0
                    detection.image_path = str(evidence)
                    detection.bbox_x_min = prediction.bbox_x_min
                    detection.bbox_y_min = prediction.bbox_y_min
                    detection.bbox_x_max = prediction.bbox_x_max
                    detection.bbox_y_max = prediction.bbox_y_max
                    detection.model_version = prediction.model_version or observation['model_version']
                    detections.append(detection)
                self._publish_detections(detections)

            self._return_and_land(goal_handle, len(detections), route)
            self._set_state('COMPLETE', goal_handle, len(detections))
            detected_types = {item.target_type for item in detections}
            missed_types = sorted(allowed_types - detected_types)
            mission_message = 'Mission completed'
            if missed_types:
                mission_message += '; camera perception missed: ' + ', '.join(missed_types)
            report = self._write_report(
                mission_id, scenario_name, detections, True, mission_message,
                planned_route, allowed_types, perception_observations,
            )
            goal_handle.succeed()
            result = SurveyMission.Result()
            result.success, result.detections, result.report_path = True, len(detections), str(report)
            result.message = f'Farm survey complete: {scenario_name}; {mission_message}'
            return result
        except MissionCanceled:
            self._set_state('EMERGENCY', goal_handle, len(detections))
            self._attempt_return_and_land()
            report = self._write_report(
                mission_id, scenario_name, detections, False,
                'Mission canceled; returned home', planned_route,
                allowed_types, perception_observations,
            )
            goal_handle.canceled()
            result = SurveyMission.Result()
            result.success, result.detections, result.report_path = False, len(detections), str(report)
            result.message = 'Mission canceled; returned home'
            return result
        except Exception as error:
            self.get_logger().error(f'Mission failed: {error}')
            self._set_state('EMERGENCY', goal_handle, len(detections))
            self._attempt_return_and_land()
            report = self._write_report(
                mission_id, scenario_name, detections, False, str(error), planned_route,
                allowed_types, perception_observations,
            )
            goal_handle.abort()
            result = SurveyMission.Result()
            result.success, result.detections, result.report_path = False, len(detections), str(report)
            result.message = f'Mission failed; returned home: {error}'
            return result
        finally:
            with self._mission_lock:
                self._mission_active = False

    def _wait_for_position(self, x, y, z, goal_handle=None, timeout=30.0,
                           progress_start=0.0, progress_end=1.0):
        try:
            self.navigator.timeout = timeout
            self.navigator.goto(
                (x, y, z),
                cancel_requested=lambda: goal_handle is not None and goal_handle.is_cancel_requested,
                progress_start=progress_start,
                progress_end=progress_end,
            )
        except NavigationCanceled as error:
            raise MissionCanceled() from error

    def _return_and_land(self, goal_handle=None, detections=0, route=None):
        segment_count = len(route) - 1 if route else 1
        self._set_state('RETURN', goal_handle, detections)
        self._wait_for_position(
            self.home_x, self.home_y, self.takeoff_altitude,
            progress_start=max(0.0, (segment_count - 2) / segment_count),
            progress_end=max(0.0, (segment_count - 1) / segment_count),
        )
        self._set_state('LAND', goal_handle, detections)
        self._wait_for_position(
            self.home_x, self.home_y, self.landing_altitude,
            progress_start=max(0.0, (segment_count - 1) / segment_count),
            progress_end=1.0,
        )

    def _build_route(self, waypoints):
        """Build the complete 3-D flight route without changing scene coordinates."""
        route = [(self.home_x, self.home_y, self.landing_altitude)]
        route.append((self.home_x, self.home_y, self.takeoff_altitude))
        route.extend((float(item['x']), float(item['y']), float(item.get('z', self.takeoff_altitude)))
                     for item in waypoints)
        route.append((self.home_x, self.home_y, self.takeoff_altitude))
        route.append((self.home_x, self.home_y, self.landing_altitude))
        self.navigator.validate_route(route)
        return route

    def _publish_route(self, route):
        message = NavPath()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'farm_map'
        for x, y, z in route:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        self.path_pub.publish(message)

    def _attempt_return_and_land(self):
        try:
            self._return_and_land()
        except Exception as error:
            self._set_state('EMERGENCY')
            self.get_logger().error(f'Unable to complete return-to-home: {error}')

    def _publish_navigation_progress(self, value):
        message = Float32()
        message.data = max(0.0, min(1.0, float(value)))
        self.navigation_progress_pub.publish(message)

    def _observe_camera(self, allowed_types):
        """Collect fresh camera predictions for one inspection waypoint."""
        deadline = time.monotonic() + self.camera_observation_window_sec
        with self._camera_detection_condition:
            sequence = self._camera_detection_sequence
            best_by_type = {}
            model_versions = set()
            latencies = []
            inference_errors = []
            message_count = 0
            prediction_count = 0
            while time.monotonic() < deadline:
                remaining = deadline - time.monotonic()
                while self._camera_detection_sequence <= sequence and remaining > 0:
                    self._camera_detection_condition.wait(timeout=min(remaining, 0.25))
                    remaining = deadline - time.monotonic()
                if self._camera_detection_sequence <= sequence:
                    continue
                sequence = self._camera_detection_sequence
                message = self._latest_camera_detections
                if message is None:
                    continue
                message_count += 1
                if message.model_version:
                    model_versions.add(message.model_version)
                latencies.append(float(message.inference_latency_ms))
                if not message.inference_ok:
                    inference_errors.append(message.inference_error or 'camera inference failed')
                for prediction in message.detections:
                    if prediction.target_type not in allowed_types:
                        continue
                    if prediction.confidence < self.camera_confidence_threshold:
                        continue
                    prediction_count += 1
                    current = best_by_type.get(prediction.target_type)
                    if current is None or prediction.confidence > current.confidence:
                        best_by_type[prediction.target_type] = prediction
                    if prediction.model_version:
                        model_versions.add(prediction.model_version)
        return list(best_by_type.values()), {
            'source_topic': self.camera_detection_topic,
            'model_version': ','.join(sorted(model_versions)),
            'message_count': message_count,
            'prediction_count': prediction_count,
            'inference_errors': inference_errors,
            'inference_latency_ms': latencies,
            'detected_types': sorted(best_by_type),
        }

    def _capture(self, mission_id, target_id):
        path = self.artifact_dir / mission_id / f'{target_id}.png'
        path.parent.mkdir(parents=True, exist_ok=True)
        if self.latest_image is None:
            raise RuntimeError('No rendered Gazebo camera frame is available')
        write_rgb_png(
            path,
            int(self.latest_image.width),
            int(self.latest_image.height),
            image_to_rgb(self.latest_image),
        )
        return path

    def _publish_detections(self, detections):
        message = TargetDetectionArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'farm_map'
        message.detections = detections
        self.detection_pub.publish(message)

    def _write_report(
        self, mission_id, scenario_name, detections, success, message, route,
        expected_types=(), perception_observations=(),
    ):
        path = self.artifact_dir / mission_id / 'report.json'
        path.parent.mkdir(parents=True, exist_ok=True)
        expected_types = set(expected_types)
        detected_types = {item.target_type for item in detections}
        model_versions = sorted({item.model_version for item in detections if item.model_version})
        report = {
            'mission_id': mission_id,
            'scenario': scenario_name,
            'success': success,
            'message': message,
            'planned_route': [
                {'x': x, 'y': y, 'z': z} for x, y, z in route
            ],
            'replan_events': list(self.navigator.replan_events),
            'state_history': self._state_history,
            'perception': {
                'mode': 'camera',
                'source_topic': self.camera_detection_topic,
                'confidence_threshold': self.camera_confidence_threshold,
                'expected_target_types': sorted(expected_types),
                'detected_target_types': sorted(detected_types),
                'missed_target_types': sorted(expected_types - detected_types),
                'model_versions': model_versions,
                'observations': perception_observations,
            },
            'detections': [
                {'target_id': item.target_id, 'target_type': item.target_type,
                 'confidence': item.confidence, 'image_path': item.image_path,
                 'model_version': item.model_version,
                 'bbox_pixels': {
                     'x_min': item.bbox_x_min, 'y_min': item.bbox_y_min,
                     'x_max': item.bbox_x_max, 'y_max': item.bbox_y_max,
                 }}
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

import json
import math
import os
import threading
import time
from pathlib import Path

import rclpy
import yaml
from drone_observer_msgs.action import SurveyMission
from drone_observer_msgs.msg import TargetDetection, TargetDetectionArray, TargetTruthArray
from geometry_msgs.msg import PoseStamped
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
from .navigation import FlightNavigator, Geofence, NavigationCanceled, ObstacleZone


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
        self.navigation_progress_pub = self.create_publisher(Float32, '/drone/navigation_progress', 10)
        self.navigator = FlightNavigator(
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
        self.truth = {}
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
        self.create_subscription(TargetTruthArray, '/drone/target_truth', self.on_truth, 10, callback_group=self.callback_group)
        self.create_subscription(Image, '/drone/camera/image_raw', self.on_image, 10, callback_group=self.callback_group)
        self.create_subscription(Odometry, '/drone/odom', self.on_odom, 10, callback_group=self.callback_group)
        self.server = ActionServer(self, SurveyMission, '/drone/survey', self.execute,
                                   goal_callback=self.accept_goal, cancel_callback=self.accept_cancel,
                                   callback_group=self.callback_group)
        self.get_logger().info(f'Farm observer ready; default scenario: {self.default_scenario}')

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
        self._state_history = []
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
                visible = self._targets_near(x, y, allowed_types)
                if not visible:
                    self._set_state('INSPECT', goal_handle, len(detections))
                for target in visible:
                    self._set_state('INSPECT', goal_handle, len(detections), target.id)
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

            self._return_and_land(goal_handle, len(detections), route)
            self._set_state('COMPLETE', goal_handle, len(detections))
            report = self._write_report(mission_id, scenario_name, detections, True, 'Mission completed', planned_route)
            goal_handle.succeed()
            result = SurveyMission.Result()
            result.success, result.detections, result.report_path = True, len(detections), str(report)
            result.message = f'Farm survey complete: {scenario_name}'
            return result
        except MissionCanceled:
            self._set_state('EMERGENCY', goal_handle, len(detections))
            self._attempt_return_and_land()
            report = self._write_report(
                mission_id, scenario_name, detections, False,
                'Mission canceled; returned home', planned_route,
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
            report = self._write_report(mission_id, scenario_name, detections, False, str(error), planned_route)
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

    def _targets_near(self, x, y, allowed_types):
        return [target for target in self.truth.values()
                if target.target_type in allowed_types and math.hypot(target.pose.position.x - x, target.pose.position.y - y) < 8.0]

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

    def _write_report(self, mission_id, scenario_name, detections, success, message, route):
        path = self.artifact_dir / mission_id / 'report.json'
        path.parent.mkdir(parents=True, exist_ok=True)
        report = {
            'mission_id': mission_id,
            'scenario': scenario_name,
            'success': success,
            'message': message,
            'planned_route': [
                {'x': x, 'y': y, 'z': z} for x, y, z in route
            ],
            'state_history': self._state_history,
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

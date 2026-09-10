import math
import threading
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import Bool

from .navigation import FlightNavigator, NavigationCanceled


class Nav2Navigator:
    """Use Nav2 for planar motion and the existing controller for altitude."""

    def __init__(self, node, publish_setpoint, get_pose, on_progress=None,
                 geofence=None, obstacles=None, tolerance=0.5, timeout=30.0):
        self.node = node
        self.get_pose = get_pose
        self.tolerance = tolerance
        self.action_client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        self.active_pub = node.create_publisher(Bool, '/drone/nav2_active', 10)
        self.direct = FlightNavigator(
            publish_setpoint, get_pose, on_progress, geofence, obstacles,
            tolerance=tolerance, timeout=timeout,
        )
        self._lock = threading.RLock()
        self._active_target = None
        self._last_blockers = ()

    @property
    def obstacles(self):
        return self.direct.obstacles

    @property
    def replan_events(self):
        return self.direct.replan_events

    @property
    def timeout(self):
        return self.direct.timeout

    @timeout.setter
    def timeout(self, value):
        self.direct.timeout = value

    def set_dynamic_obstacles(self, obstacles):
        self.direct.set_dynamic_obstacles(obstacles)
        with self._lock:
            active_target = self._active_target
            current = self.get_pose()
            blockers = tuple(
                obstacle.zone_id for obstacle in obstacles or ()
                if active_target and current and obstacle.intersects(current, active_target)
            )
            if active_target and blockers and blockers != self._last_blockers:
                self.direct.replan_events.append({
                    'timestamp': time.time(),
                    'from': list(current),
                    'to': list(active_target),
                    'obstacles': list(blockers),
                    'planner': 'nav2',
                })
            self._last_blockers = blockers

    def reset_replan_events(self):
        self.direct.reset_replan_events()
        with self._lock:
            self._last_blockers = ()

    def validate_route(self, route):
        self.direct.validate_route(route)

    def plan_route(self, route):
        # Keep the report and mission path useful before Nav2 computes its live plan.
        return self.direct.plan_route(route)

    def goto(self, target, cancel_requested=None, progress_start=0.0, progress_end=1.0):
        target = tuple(float(value) for value in target)
        cancel_requested = cancel_requested or (lambda: False)
        current = self.get_pose() or target
        horizontal_target = (target[0], target[1], current[2])
        horizontal_distance = math.dist(current[:2], horizontal_target[:2])
        if horizontal_distance > self.tolerance:
            split = progress_start + (progress_end - progress_start) * 0.9
            self._goto_planar(horizontal_target, cancel_requested, progress_start, split)
            self.direct.goto(
                target, cancel_requested, split, progress_end,
            )
        else:
            self.direct.goto(target, cancel_requested, progress_start, progress_end)

    def _goto_planar(self, target, cancel_requested, progress_start, progress_end):
        self.direct.geofence.validate_pose(target)
        if not self.action_client.wait_for_server(timeout_sec=self.timeout):
            raise TimeoutError('Nav2 NavigateToPose action server is unavailable')

        current = self.get_pose() or target
        # The direct planner expands configured static and runtime obstacle
        # zones into safe intermediate points.  Use those points as Nav2
        # sub-goals instead of asking Nav2 to cross a blocked straight line.
        path = self.direct.plan_segment(current, target)
        lengths = [math.dist(left[:2], right[:2]) for left, right in zip(path, path[1:])]
        total_distance = sum(lengths) or 1.0

        self._publish_active(True)
        with self._lock:
            self._active_target = target
            self._last_blockers = ()
        try:
            traveled = 0.0
            for segment_target, segment_length in zip(path[1:], lengths):
                segment_start = progress_start + (progress_end - progress_start) * traveled / total_distance
                traveled += segment_length
                segment_end = progress_start + (progress_end - progress_start) * traveled / total_distance
                self._send_nav2_goal(
                    segment_target,
                    cancel_requested,
                    segment_start,
                    segment_end,
                    segment_length,
                )
            self.direct._report_progress(progress_end)
        finally:
            with self._lock:
                self._active_target = None
                self._last_blockers = ()
            self._publish_active(False)

    def _send_nav2_goal(self, target, cancel_requested, progress_start, progress_end, segment_length):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = target[0]
        goal.pose.pose.position.y = target[1]
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.w = 1.0
        send_future = self.action_client.send_goal_async(
            goal, feedback_callback=lambda message: self._on_feedback(
                message, segment_length or 1.0, progress_start, progress_end,
            ),
        )
        goal_handle = self._wait_future(send_future, cancel_requested)
        if not goal_handle.accepted:
            raise RuntimeError(f'Nav2 rejected goal at ({target[0]}, {target[1]})')
        result_future = goal_handle.get_result_async()
        result = self._wait_future(result_future, cancel_requested)
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f'Nav2 failed to reach ({target[0]}, {target[1]}); status={result.status}')

    def _on_feedback(self, message, initial_distance, progress_start, progress_end):
        remaining = max(0.0, float(message.feedback.distance_remaining))
        fraction = 1.0 - min(1.0, remaining / initial_distance)
        self.direct._report_progress(
            progress_start + (progress_end - progress_start) * fraction,
        )

    def _wait_future(self, future, cancel_requested):
        deadline = time.monotonic() + self.timeout
        while not future.done():
            if cancel_requested():
                raise NavigationCanceled()
            if time.monotonic() >= deadline:
                raise TimeoutError('Timed out waiting for Nav2')
            time.sleep(0.05)
        return future.result()

    def _publish_active(self, active):
        message = Bool()
        message.data = bool(active)
        self.active_pub.publish(message)

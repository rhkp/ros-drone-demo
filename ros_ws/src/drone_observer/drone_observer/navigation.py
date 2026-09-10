import math
import threading
import time
from dataclasses import dataclass


class NavigationCanceled(Exception):
    """Raised when a flight goal is canceled while moving."""


class NavigationReplan(Exception):
    """Raised internally when a newly reported obstacle blocks a segment."""


@dataclass(frozen=True)
class Geofence:
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    min_z: float
    max_z: float

    @classmethod
    def from_mapping(cls, mapping):
        values = mapping or {}
        geofence = cls(
            float(values.get('min_x', -40.0)),
            float(values.get('max_x', 40.0)),
            float(values.get('min_y', -30.0)),
            float(values.get('max_y', 30.0)),
            float(values.get('min_z', 0.0)),
            float(values.get('max_z', 25.0)),
        )
        if geofence.min_x >= geofence.max_x or geofence.min_y >= geofence.max_y or geofence.min_z >= geofence.max_z:
            raise ValueError('Navigation geofence minimums must be less than maximums')
        return geofence

    def validate_pose(self, pose):
        x, y, z = pose
        if not (self.min_x <= x <= self.max_x and
                self.min_y <= y <= self.max_y and
                self.min_z <= z <= self.max_z):
            raise ValueError(f'Navigation target ({x}, {y}, {z}) is outside the configured geofence')

    def validate_route(self, route):
        for pose in route:
            self.validate_pose(pose)


@dataclass(frozen=True)
class ObstacleZone:
    zone_id: str
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    min_z: float
    max_z: float
    margin: float = 1.0

    @classmethod
    def from_mapping(cls, mapping):
        values = mapping or {}
        zone = cls(
            str(values.get('id', 'unnamed-zone')),
            float(values['min_x']),
            float(values['max_x']),
            float(values['min_y']),
            float(values['max_y']),
            float(values.get('min_z', 0.0)),
            float(values.get('max_z', 25.0)),
            float(values.get('margin', 1.0)),
        )
        if (zone.min_x >= zone.max_x or zone.min_y >= zone.max_y or
                zone.min_z >= zone.max_z or zone.margin < 0.0):
            raise ValueError(f'Invalid obstacle zone: {zone.zone_id}')
        return zone

    def _z_intersects(self, start, end):
        return max(min(start[2], end[2]), self.min_z) <= min(max(start[2], end[2]), self.max_z)

    def _contains_xy(self, pose):
        return self.min_x < pose[0] < self.max_x and self.min_y < pose[1] < self.max_y

    @staticmethod
    def _crosses_open_rectangle(start, end, min_x, max_x, min_y, max_y):
        epsilon = 1e-7
        min_x += epsilon
        max_x -= epsilon
        min_y += epsilon
        max_y -= epsilon
        dx, dy = end[0] - start[0], end[1] - start[1]
        lower, upper = 0.0, 1.0
        for origin, delta, lower_bound, upper_bound in (
                (start[0], dx, min_x, max_x), (start[1], dy, min_y, max_y)):
            if abs(delta) < epsilon:
                if lower_bound < origin < upper_bound:
                    continue
                return False
            first = (lower_bound - origin) / delta
            second = (upper_bound - origin) / delta
            lower = max(lower, min(first, second))
            upper = min(upper, max(first, second))
            if lower >= upper:
                return False
        return True

    def intersects(self, start, end):
        if not self._z_intersects(start, end):
            return False
        return self._crosses_open_rectangle(
            start, end,
            self.min_x, self.max_x, self.min_y, self.max_y,
        )

    def detour_for_segment(self, start, end):
        if not self.intersects(start, end):
            return None
        start_inside = self._contains_xy(start)
        if self._contains_xy(end):
            raise ValueError(f'Route enters obstacle zone: {self.zone_id}')

        min_x, max_x = self.min_x - self.margin, self.max_x + self.margin
        min_y, max_y = self.min_y - self.margin, self.max_y + self.margin
        z = max(start[2], end[2])
        corners = [
            (min_x, min_y, z), (min_x, max_y, z),
            (max_x, min_y, z), (max_x, max_y, z),
        ]
        nodes = [tuple(start), tuple(end)] + corners
        edges = {index: [] for index in range(len(nodes))}
        for left in range(len(nodes)):
            for right in range(left + 1, len(nodes)):
                crosses_zone = self._crosses_open_rectangle(
                    nodes[left], nodes[right], min_x, max_x, min_y, max_y,
                )
                escape_edge = start_inside and left == 0 and right >= 2
                if crosses_zone and not escape_edge:
                    continue
                distance = math.dist(nodes[left], nodes[right])
                edges[left].append((right, distance))
                edges[right].append((left, distance))

        distances = [math.inf] * len(nodes)
        previous = [None] * len(nodes)
        distances[0] = 0.0
        unvisited = set(range(len(nodes)))
        while unvisited:
            current = min(unvisited, key=lambda index: distances[index])
            unvisited.remove(current)
            if current == 1 or distances[current] == math.inf:
                break
            for neighbor, distance in edges[current]:
                candidate = distances[current] + distance
                if candidate < distances[neighbor]:
                    distances[neighbor] = candidate
                    previous[neighbor] = current
        if distances[1] == math.inf:
            raise ValueError(f'Unable to route around obstacle zone: {self.zone_id}')

        path = []
        current = 1
        while current is not None:
            path.append(nodes[current])
            current = previous[current]
        return list(reversed(path))


class FlightNavigator:
    """Small planner/controller boundary that can later be backed by Nav2."""

    def __init__(self, publish_setpoint, get_pose, on_progress=None, geofence=None,
                 obstacles=None,
                 tolerance=0.5, update_period=0.1, timeout=30.0):
        self.publish_setpoint = publish_setpoint
        self.get_pose = get_pose
        self.on_progress = on_progress or (lambda _value: None)
        self.geofence = geofence or Geofence.from_mapping({})
        self.static_obstacles = tuple(obstacles or ())
        self.dynamic_obstacles = ()
        self._obstacle_lock = threading.RLock()
        self.replan_events = []
        self.tolerance = tolerance
        self.update_period = update_period
        self.timeout = timeout

    @property
    def obstacles(self):
        with self._obstacle_lock:
            return self.static_obstacles + self.dynamic_obstacles

    def set_dynamic_obstacles(self, obstacles):
        """Replace the current runtime obstacle snapshot atomically."""
        with self._obstacle_lock:
            self.dynamic_obstacles = tuple(obstacles or ())

    def reset_replan_events(self):
        with self._obstacle_lock:
            self.replan_events = []

    def validate_route(self, route):
        self.geofence.validate_route(route)

    def plan_segment(self, start, target):
        path = [tuple(start), tuple(target)]
        for obstacle in self.obstacles:
            planned = []
            for left, right in zip(path, path[1:]):
                detour = obstacle.detour_for_segment(left, right)
                if detour:
                    planned.extend(detour[:-1])
                else:
                    planned.append(left)
            planned.append(path[-1])
            path = planned
        return path

    def plan_route(self, route):
        self.validate_route(route)
        planned = [tuple(route[0])]
        for left, right in zip(route, route[1:]):
            segment = self.plan_segment(left, right)
            planned.extend(segment[1:])
        return planned

    def goto(self, target, cancel_requested=None, progress_start=0.0, progress_end=1.0):
        target = tuple(float(value) for value in target)
        self.geofence.validate_pose(target)
        cancel_requested = cancel_requested or (lambda: False)
        self._progress_value = progress_start
        replan_count = 0
        deadline = time.monotonic() + self.timeout
        while True:
            if time.monotonic() >= deadline:
                raise TimeoutError(f'Timed out planning route to navigation target {target}')
            start = self.get_pose() or target
            path = self.plan_segment(start, target)
            start_inside_obstacle = any(
                obstacle._contains_xy(start) for obstacle in self.obstacles
            )
            lengths = [math.dist(left, right) for left, right in zip(path, path[1:])]
            total_length = sum(lengths) or 1.0
            traveled = 0.0
            try:
                for index, waypoint in enumerate(path[1:]):
                    segment_start = progress_start + (progress_end - progress_start) * traveled / total_length
                    traveled += lengths[index]
                    segment_end = progress_start + (progress_end - progress_start) * traveled / total_length
                    self._move_to(
                        waypoint, cancel_requested, segment_start, segment_end,
                        allow_escape=start_inside_obstacle and index == 0,
                    )
                return
            except NavigationReplan as error:
                replan_count += 1
                if replan_count > 20:
                    raise TimeoutError(f'Too many navigation replans while reaching {target}') from error
                current = self.get_pose() or start
                with self._obstacle_lock:
                    blockers = [obstacle.zone_id for obstacle in self.dynamic_obstacles
                                if obstacle.intersects(current, target)]
                    self.replan_events.append({
                        'timestamp': time.time(),
                        'from': list(current),
                        'to': list(target),
                        'obstacles': blockers,
                    })

    def _move_to(self, target, cancel_requested, progress_start, progress_end,
                 allow_escape=False):
        initial_pose = self.get_pose()
        initial_distance = math.dist(initial_pose, target) if initial_pose else 0.0
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            if cancel_requested():
                raise NavigationCanceled()
            pose = self.get_pose()
            blocked = False
            if pose:
                for obstacle in self.obstacles:
                    if not obstacle.intersects(pose, target):
                        continue
                    if allow_escape and obstacle._contains_xy(pose):
                        continue
                    blocked = True
                    break
            if blocked:
                raise NavigationReplan(f'Navigation segment to {target} is blocked')
            self.publish_setpoint(*target)
            if pose and math.dist(pose, target) < self.tolerance:
                self._report_progress(progress_end)
                return
            if pose and initial_distance > self.tolerance:
                fraction = 1.0 - min(1.0, math.dist(pose, target) / initial_distance)
                self._report_progress(progress_start + (progress_end - progress_start) * fraction)
            time.sleep(self.update_period)
        raise TimeoutError(f'Timed out reaching navigation target {target}')

    def _report_progress(self, value):
        self._progress_value = max(self._progress_value, float(value))
        self.on_progress(self._progress_value)

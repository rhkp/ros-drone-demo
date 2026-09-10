import math
import time
from dataclasses import dataclass


class NavigationCanceled(Exception):
    """Raised when a flight goal is canceled while moving."""


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


class FlightNavigator:
    """Small planner/controller boundary that can later be backed by Nav2."""

    def __init__(self, publish_setpoint, get_pose, on_progress=None, geofence=None,
                 tolerance=0.5, update_period=0.1, timeout=30.0):
        self.publish_setpoint = publish_setpoint
        self.get_pose = get_pose
        self.on_progress = on_progress or (lambda _value: None)
        self.geofence = geofence or Geofence.from_mapping({})
        self.tolerance = tolerance
        self.update_period = update_period
        self.timeout = timeout

    def validate_route(self, route):
        self.geofence.validate_route(route)

    def goto(self, target, cancel_requested=None, progress_start=0.0, progress_end=1.0):
        target = tuple(float(value) for value in target)
        self.geofence.validate_pose(target)
        cancel_requested = cancel_requested or (lambda: False)
        initial_pose = self.get_pose()
        initial_distance = math.dist(initial_pose, target) if initial_pose else 0.0
        deadline = time.monotonic() + self.timeout

        while time.monotonic() < deadline:
            if cancel_requested():
                raise NavigationCanceled()
            self.publish_setpoint(*target)
            pose = self.get_pose()
            if pose and math.dist(pose, target) < self.tolerance:
                self.on_progress(progress_end)
                return
            if pose and initial_distance > self.tolerance:
                fraction = 1.0 - min(1.0, math.dist(pose, target) / initial_distance)
                self.on_progress(progress_start + (progress_end - progress_start) * fraction)
            time.sleep(self.update_period)
        raise TimeoutError(f'Timed out reaching navigation target {target}')

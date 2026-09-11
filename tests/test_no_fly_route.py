import sys
import unittest
from pathlib import Path


SOURCE_ROOT = Path(__file__).parents[1] / 'ros_ws' / 'src' / 'drone_observer'
sys.path.insert(0, str(SOURCE_ROOT))

from drone_observer.navigation import FlightNavigator, Geofence, ObstacleZone  # noqa: E402


class NoFlyRouteTest(unittest.TestCase):
    def test_all_scenario_route_detours_around_restricted_zone(self):
        zone = ObstacleZone(
            'north_access_restriction', -2.0, 4.0, 11.0, 15.0, 8.0, 20.0, 2.0,
        )
        navigator = FlightNavigator(
            lambda *_: None,
            lambda: None,
            geofence=Geofence(-35.0, 35.0, -25.0, 25.0, 0.0, 20.0),
            obstacles=[zone],
        )
        route = [
            (10.0, 16.0, 0.6),
            (10.0, 16.0, 12.0),
            (-18.0, 10.0, 12.0),
            (0.0, 10.0, 12.0),
            (17.0, 8.0, 12.0),
            (-4.0, 0.0, 14.0),
            (-22.0, 3.0, 14.0),
            (18.0, -10.0, 12.0),
            (10.0, 16.0, 12.0),
            (10.0, 16.0, 0.6),
        ]
        planned = navigator.plan_route(route)

        self.assertGreater(len(planned), len(route))
        self.assertTrue(
            all(not zone.intersects(start, end) for start, end in zip(planned, planned[1:]))
        )
        self.assertIn((6.0, 17.0, 12.0), planned)
        self.assertIn((-4.0, 17.0, 12.0), planned)


if __name__ == '__main__':
    unittest.main()

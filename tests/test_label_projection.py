import sys
from pathlib import Path


SOURCE_ROOT = Path(__file__).parents[1] / 'ros_ws' / 'src' / 'drone_observer'
sys.path.insert(0, str(SOURCE_ROOT))

from drone_observer.label_projection import normalize_box, project_target_box  # noqa: E402


def test_projection_is_centered_for_target_below_drone():
    camera = {'width': 320, 'height': 240, 'fx': 277.1, 'fy': 277.1, 'cx': 160, 'cy': 120}
    box = project_target_box(
        {'x': 0, 'y': 0, 'z': 0}, {'x': 2, 'y': 2, 'z': 2},
        (0, 0, 10), camera,
    )
    assert box['center_x'] == 160.0
    assert box['center_y'] == 120.0
    assert box['width'] > 0
    assert box['height'] > 0


def test_projection_clips_objects_at_image_edge():
    camera = {'width': 320, 'height': 240, 'fx': 277.1, 'fy': 277.1, 'cx': 160, 'cy': 120}
    box = project_target_box(
        {'x': 8, 'y': 0, 'z': 0}, {'x': 4, 'y': 4, 'z': 2},
        (0, 0, 10), camera,
    )
    assert 0.0 < box['x_min'] < 320.0
    assert box['x_max'] == 320.0


def test_gazebo_downward_camera_swaps_and_flips_world_axes():
    camera = {
        'width': 320, 'height': 240, 'fx': 277.1, 'fy': 277.1,
        'cx': 160, 'cy': 120,
        'image_x_axis': 'y', 'image_x_sign': -1,
        'image_y_axis': 'x', 'image_y_sign': -1,
    }
    box = project_target_box(
        {'x': -18, 'y': 10, 'z': 1}, {'x': 3, 'y': 2, 'z': 2.2},
        (-14.7443, 12.5397, 11.5720), {**camera, 'offset_z': 0.36},
    )
    assert box['center_x'] > 160.0
    assert box['center_y'] > 120.0


def test_yolo_normalization():
    normalized = normalize_box(
        {'center_x': 160, 'center_y': 120, 'width': 80, 'height': 60}, 320, 240,
    )
    assert normalized == {'center_x': 0.5, 'center_y': 0.5, 'width': 0.25, 'height': 0.25}

"""Dependency-free projection helpers for synthetic detector labels.

The current farm camera is a fixed, nadir-facing camera. Keeping projection in
its own module makes the geometry testable without a running ROS graph and makes
the camera convention visible in the dataset metadata.
"""

from itertools import product


def project_target_box(target, dimensions, drone_pose, camera, vertical_sign=1):
    """Project an axis-aligned farm target into a clipped pixel bounding box.

    ``camera`` contains ``fx``, ``fy``, ``cx`` and ``cy``. The current Gazebo
    sensor is pitched 90 degrees, so its image axes are explicitly configurable
    instead of assuming world X/Y map directly to image X/Y. The returned box
    uses the ``x_min, y_min, x_max, y_max`` convention.
    """
    width, height = int(camera['width']), int(camera['height'])
    fx, fy = float(camera['fx']), float(camera['fy'])
    cx, cy = float(camera['cx']), float(camera['cy'])
    drone_x, drone_y, drone_z = (float(value) for value in drone_pose)
    target_x, target_y, target_z = (
        float(target['x']), float(target['y']), float(target.get('z', 0.0))
    )
    size_x, size_y, size_z = (float(dimensions[axis]) for axis in ('x', 'y', 'z'))
    camera_z = drone_z - float(camera.get('offset_z', 0.36))
    image_x_axis = camera.get('image_x_axis', 'x')
    image_y_axis = camera.get('image_y_axis', 'y')
    image_x_sign = float(camera.get('image_x_sign', 1))
    image_y_sign = float(camera.get('image_y_sign', vertical_sign))
    points = []
    for offset_x, offset_y, offset_z in product((-0.5, 0.5), repeat=3):
        world_x = target_x + offset_x * size_x
        world_y = target_y + offset_y * size_y
        world_z = target_z + offset_z * size_z
        depth = camera_z - world_z
        if depth <= 0.01:
            continue
        deltas = {'x': world_x - drone_x, 'y': world_y - drone_y}
        pixel_x = cx + image_x_sign * fx * deltas[image_x_axis] / depth
        pixel_y = cy + image_y_sign * fy * deltas[image_y_axis] / depth
        points.append((pixel_x, pixel_y))
    if not points:
        return None
    x_min = max(0.0, min(point[0] for point in points))
    y_min = max(0.0, min(point[1] for point in points))
    x_max = min(float(width), max(point[0] for point in points))
    y_max = min(float(height), max(point[1] for point in points))
    if x_max - x_min < 1.0 or y_max - y_min < 1.0:
        return None
    return {
        'x_min': round(x_min, 3),
        'y_min': round(y_min, 3),
        'x_max': round(x_max, 3),
        'y_max': round(y_max, 3),
        'width': round(x_max - x_min, 3),
        'height': round(y_max - y_min, 3),
        'center_x': round((x_min + x_max) / 2.0, 3),
        'center_y': round((y_min + y_max) / 2.0, 3),
    }


def normalize_box(box, width, height):
    """Convert a pixel box into YOLO's normalized center/width/height format."""
    return {
        'center_x': round(box['center_x'] / float(width), 6),
        'center_y': round(box['center_y'] / float(height), 6),
        'width': round(box['width'] / float(width), 6),
        'height': round(box['height'] / float(height), 6),
    }

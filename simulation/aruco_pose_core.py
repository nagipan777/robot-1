"""ROS/OpenCV-independent fixed-camera ArUco pose geometry and freshness gate."""

import math


def select_marker_observation(observations, marker_id):
    if not isinstance(observations, list):
        raise ValueError("aruco_detections must be a list")
    for observation in observations:
        if isinstance(observation, dict) and observation.get("marker_id") == marker_id:
            return observation
    return None


def _finite(value, name):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("{} must be finite".format(name))
    return value


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def transform_pixel(homography, pixel):
    if len(homography) != 9:
        raise ValueError("homography must contain nine values")
    h = [_finite(value, "homography") for value in homography]
    u, v = (_finite(pixel[0], "pixel x"), _finite(pixel[1], "pixel y"))
    denominator = h[6] * u + h[7] * v + h[8]
    if abs(denominator) < 1e-12:
        raise ValueError("homography denominator is zero")
    return (
        (h[0] * u + h[1] * v + h[2]) / denominator,
        (h[3] * u + h[4] * v + h[5]) / denominator,
    )


def correct_height(floor_intersection, camera_ground, camera_height, marker_height):
    camera_height = _finite(camera_height, "camera_height")
    marker_height = _finite(marker_height, "marker_height")
    if camera_height <= 0.0 or not 0.0 <= marker_height < camera_height:
        raise ValueError("require 0 <= marker_height < camera_height")
    cx, cy = map(float, camera_ground)
    gx, gy = floor_intersection
    scale = (camera_height - marker_height) / camera_height
    return cx + scale * (gx - cx), cy + scale * (gy - cy)


def marker_pose(
    corners,
    homography,
    *,
    camera_ground,
    camera_height,
    marker_height,
    marker_to_robot_forward=0.0,
    marker_to_robot_left=0.0,
    yaw_offset=0.0,
    min_pixel_area=100.0,
    map_bounds=None,
):
    if len(corners) != 4 or any(len(point) != 2 for point in corners):
        raise ValueError("four 2D marker corners are required")
    points = [(_finite(x, "corner x"), _finite(y, "corner y")) for x, y in corners]
    signed_area = 0.5 * sum(
        points[index][0] * points[(index + 1) % 4][1]
        - points[(index + 1) % 4][0] * points[index][1]
        for index in range(4)
    )
    if abs(signed_area) < _finite(min_pixel_area, "min_pixel_area"):
        raise ValueError("marker pixel area is too small")

    mapped = [
        correct_height(
            transform_pixel(homography, point),
            camera_ground,
            camera_height,
            marker_height,
        )
        for point in points
    ]
    center_x = sum(point[0] for point in mapped) / 4.0
    center_y = sum(point[1] for point in mapped) / 4.0
    edge_x = mapped[1][0] - mapped[0][0]
    edge_y = mapped[1][1] - mapped[0][1]
    if math.hypot(edge_x, edge_y) < 1e-9:
        raise ValueError("marker heading edge collapsed")
    yaw = normalize_angle(math.atan2(edge_y, edge_x) + _finite(yaw_offset, "yaw_offset"))

    forward = _finite(marker_to_robot_forward, "marker_to_robot_forward")
    left = _finite(marker_to_robot_left, "marker_to_robot_left")
    robot_x = center_x + math.cos(yaw) * forward - math.sin(yaw) * left
    robot_y = center_y + math.sin(yaw) * forward + math.cos(yaw) * left
    if map_bounds is not None:
        if len(map_bounds) != 4:
            raise ValueError("map_bounds must be [min_x, max_x, min_y, max_y]")
        min_x, max_x, min_y, max_y = map(float, map_bounds)
        if not min_x <= robot_x <= max_x or not min_y <= robot_y <= max_y:
            raise ValueError("pose is outside map bounds")
    return robot_x, robot_y, yaw


class PoseFreshnessGate:
    def __init__(self, max_age=0.5, min_consecutive=3):
        if max_age <= 0.0 or min_consecutive < 1:
            raise ValueError("invalid freshness gate configuration")
        self.max_age = float(max_age)
        self.min_consecutive = int(min_consecutive)
        self.last_timestamp = None
        self.count = 0

    def observe(self, timestamp, now):
        timestamp = _finite(timestamp, "timestamp")
        now = _finite(now, "now")
        age = now - timestamp
        if age < -0.1 or age > self.max_age:
            self.reset()
            return False, "stale_or_future"
        if self.last_timestamp is not None and timestamp <= self.last_timestamp:
            self.reset()
            return False, "non_monotonic"
        self.last_timestamp = timestamp
        self.count += 1
        return self.count >= self.min_consecutive, "accepted" if self.count >= self.min_consecutive else "warming_up"

    def missing(self):
        self.reset()
        return False, "marker_lost"

    def reset(self):
        self.last_timestamp = None
        self.count = 0


class StreamTimeoutGate:
    """Track arrival of camera payloads using a monotonic wall clock."""

    def __init__(self, max_age=0.5):
        if max_age <= 0.0:
            raise ValueError("invalid stream timeout configuration")
        self.max_age = float(max_age)
        self.last_frame_at = None
        self.timeout_reported = False

    def frame_received(self, now):
        self.last_frame_at = _finite(now, "frame receipt time")
        self.timeout_reported = False

    def check(self, now):
        now = _finite(now, "timeout check time")
        if self.last_frame_at is None or self.timeout_reported:
            return False
        if now - self.last_frame_at <= self.max_age:
            return False
        self.timeout_reported = True
        return True

"""ROS-independent Navigation command model for hardware-test planning only."""

import math
import sys
from dataclasses import asdict, dataclass
from pathlib import Path

try:
    from differential_drive import twist_to_wheel_speeds
except ModuleNotFoundError:
    from .differential_drive import twist_to_wheel_speeds

try:
    from navigation import calculate_goal
except ModuleNotFoundError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from navigation import calculate_goal


@dataclass(frozen=True)
class NavigationUnitResult:
    case: str
    distance_m: float | None
    angle_error_rad: float | None
    linear_x_m_s: float
    angular_z_rad_s: float
    left_mm_s: int
    right_mm_s: int
    stop: bool
    reason: str
    expected_motion: str

    def row(self):
        return asdict(self)


def evaluate_navigation_case(
    case,
    *,
    robot_pose=(0.0, 0.0, 0.0),
    target=(1.0, 0.0),
    mode="approach",
    pose_fresh=True,
    target_fresh=True,
    stopped_target_latched=False,
    angular_gain=1.2,
    max_angular_speed=0.8,
    angle_tolerance=0.08,
    arrival_distance=0.15,
    linear_gain=0.4,
    max_linear_speed=0.08,
    forward_angle_tolerance=0.15,
    track_width_m=0.235,
    max_wheel_speed_mm_s=20,
):
    if mode not in ("turn_only", "approach"):
        raise ValueError("mode must be turn_only or approach")
    if not pose_fresh:
        return _stop(case, "pose_stale")
    if not target_fresh:
        return _stop(case, "target_stale")
    if stopped_target_latched:
        return _stop(case, "old_target_stopped_latch")
    if robot_pose is None or target is None:
        return _stop(case, "input_missing")

    distance, error = calculate_goal(*robot_pose, *target)
    if distance <= arrival_distance:
        return _stop(case, "target_reached", distance, error)

    linear = 0.0
    angular = 0.0
    if abs(error) > angle_tolerance:
        angular = max(-max_angular_speed, min(max_angular_speed, angular_gain * error))
    if mode == "approach" and abs(error) <= forward_angle_tolerance:
        linear = max(0.0, min(max_linear_speed, linear_gain * (distance - arrival_distance)))
    left, right = twist_to_wheel_speeds(
        linear,
        angular,
        track_width_m=track_width_m,
        max_wheel_speed_mm_s=max_wheel_speed_mm_s,
    )
    stop = left == 0 and right == 0
    if stop:
        motion = "stop"
    elif left < 0 < right:
        motion = "left_rotation"
    elif right < 0 < left:
        motion = "right_rotation"
    elif left > 0 and right > 0:
        motion = "forward"
    else:
        motion = "other"
    return NavigationUnitResult(
        case, distance, error, linear, angular, left, right, stop,
        "command" if not stop else "within_command_deadband", motion,
    )


def _stop(case, reason, distance=None, error=None):
    return NavigationUnitResult(
        case, distance, error, 0.0, 0.0, 0, 0, True, reason, "stop"
    )

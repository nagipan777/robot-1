"""ROS-independent differential-drive velocity conversion for Roomba 680."""

import math
from numbers import Real
from typing import Tuple


DEFAULT_TRACK_WIDTH_M = 0.235
DEFAULT_MAX_WHEEL_SPEED_MM_S = 50
OI_MAX_WHEEL_SPEED_MM_S = 500


def twist_to_wheel_speeds(
    linear_x: float,
    angular_z: float,
    *,
    track_width_m: float = DEFAULT_TRACK_WIDTH_M,
    max_wheel_speed_mm_s: int = DEFAULT_MAX_WHEEL_SPEED_MM_S,
) -> Tuple[int, int]:
    """Convert a planar velocity command to ``(left, right)`` wheel speeds.

    Inputs use the same units and sign convention as ``geometry_msgs/Twist``:
    metres per second for ``linear_x`` and radians per second for
    ``angular_z``. Results are integer millimetres per second, ready for
    :meth:`roomba_driver.RoombaDriver.drive_direct`.

    If either wheel exceeds the configured limit, both speeds are multiplied
    by the same scale factor before rounding, preserving their ratio as far as
    integer millimetre-per-second output permits.
    """
    linear_x = _finite_number(linear_x, "linear_x")
    angular_z = _finite_number(angular_z, "angular_z")
    track_width_m = _finite_number(track_width_m, "track_width_m")
    max_wheel_speed_mm_s = _finite_number(
        max_wheel_speed_mm_s, "max_wheel_speed_mm_s"
    )

    if track_width_m <= 0:
        raise ValueError("track_width_m must be positive")
    if not 0 < max_wheel_speed_mm_s <= OI_MAX_WHEEL_SPEED_MM_S:
        raise ValueError("max_wheel_speed_mm_s must be from 1 to 500")

    half_track = track_width_m / 2.0
    left = (linear_x - angular_z * half_track) * 1000.0
    right = (linear_x + angular_z * half_track) * 1000.0

    peak = max(abs(left), abs(right))
    if peak > max_wheel_speed_mm_s:
        scale = max_wheel_speed_mm_s / peak
        left *= scale
        right *= scale

    return int(round(left)), int(round(right))


def _finite_number(value: float, name: str) -> float:
    if not isinstance(value, Real) or isinstance(value, bool):
        raise TypeError(f"{name} must be a real number")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value

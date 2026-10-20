"""ROS-independent safety validation for the Roomba wheel-lift test."""

import math


MAX_WHEEL_SPEED_MM_S = 20
MAX_DRIVE_DURATION_S = 1.0
WATCHDOG_SILENCE_S = 0.75
ROTATION_CONFIRMATION = "WHEELS_LIFTED"
ATTENDED_FLOOR_CONFIRMATION = "ATTENDED_FLOOR_CLEARED"
DEFAULT_SUBSCRIBER_WAIT_TIMEOUT_S = 5.0


class SubscriberConnectionGate:
    """Wait finitely for ROS discovery before a timed test begins."""

    def __init__(self, timeout_s=DEFAULT_SUBSCRIBER_WAIT_TIMEOUT_S, started_at=0.0):
        if not isinstance(timeout_s, (int, float)) or not math.isfinite(timeout_s):
            raise ValueError("subscriber wait timeout must be finite")
        if timeout_s <= 0.0:
            raise ValueError("subscriber wait timeout must be positive")
        self.timeout_s = float(timeout_s)
        self.started_at = float(started_at)

    def ready(self, subscription_count, now):
        if subscription_count > 0:
            return True
        if now - self.started_at >= self.timeout_s:
            raise TimeoutError(
                "no subscriber matched within {:.1f}s".format(self.timeout_s)
            )
        return False


class WheelLiftTestProfile:
    MODES = ("stop", "rejection", "rotate", "watchdog", "floor_straight")

    def __init__(
        self,
        mode="stop",
        *,
        left_mm_s=20,
        right_mm_s=20,
        drive_duration_s=1.0,
        wheel_base_m=0.235,
        allow_rotation=False,
        confirmation="",
    ):
        if mode not in self.MODES:
            raise ValueError("mode must be one of: {}".format(", ".join(self.MODES)))
        for name, value in (
            ("left_mm_s", left_mm_s),
            ("right_mm_s", right_mm_s),
            ("drive_duration_s", drive_duration_s),
            ("wheel_base_m", wheel_base_m),
        ):
            if not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError("{} must be finite".format(name))
        if abs(left_mm_s) > MAX_WHEEL_SPEED_MM_S:
            raise ValueError("left_mm_s exceeds 20 mm/s")
        if abs(right_mm_s) > MAX_WHEEL_SPEED_MM_S:
            raise ValueError("right_mm_s exceeds 20 mm/s")
        if not 0.0 < drive_duration_s <= MAX_DRIVE_DURATION_S:
            raise ValueError("drive_duration_s must be > 0 and <= 1.0")
        if wheel_base_m <= 0.0:
            raise ValueError("wheel_base_m must be positive")
        required_confirmation = (
            ATTENDED_FLOOR_CONFIRMATION
            if mode == "floor_straight"
            else ROTATION_CONFIRMATION
        )
        if mode in ("rotate", "watchdog", "floor_straight") and not (
            allow_rotation and confirmation == required_confirmation
        ):
            raise PermissionError(
                "{} requires allow_rotation=true and confirmation={}".format(
                    mode, required_confirmation
                )
            )

        self.mode = mode
        self.left_mm_s = float(left_mm_s)
        self.right_mm_s = float(right_mm_s)
        self.drive_duration_s = float(drive_duration_s)
        self.wheel_base_m = float(wheel_base_m)

    @property
    def sends_nonzero(self):
        return self.mode != "stop" and (
            self.left_mm_s != 0.0 or self.right_mm_s != 0.0
        )

    def twist(self):
        linear_x = (self.left_mm_s + self.right_mm_s) / 2000.0
        angular_z = (
            (self.right_mm_s - self.left_mm_s)
            / 1000.0
            / self.wheel_base_m
        )
        return linear_x, angular_z

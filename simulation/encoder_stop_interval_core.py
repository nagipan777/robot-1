"""STOP-to-STOP encoder interval evaluation with no Drive capability."""

import math
from dataclasses import asdict, dataclass

try:
    from localization_core import encoder_delta
except ModuleNotFoundError:
    from .localization_core import encoder_delta


ATTENDED_DRIVE_APPROVAL = "ATTENDED_DRIVE_APPROVED"
PHYSICAL_STOP_CONFIRMATION = "PHYSICAL_STOP_CONFIRMED"


def finite_optional(value, name):
    if value is None:
        return None
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("{} must be finite".format(name))
    return result


def sign(value):
    return 0 if value == 0 else (1 if value > 0 else -1)


def validate_stop_gate(status, requested_right, requested_left, oi_mode):
    if (status, requested_right, requested_left, oi_mode) != (0x07, 0, 0, 2):
        raise RuntimeError(
            "STOP/Safe gate failed: status=0x{:02x} right={} left={} mode={}".format(
                status, requested_right, requested_left, oi_mode
            )
        )


def acquire_stopped_encoder(transport):
    """One STOP-first sample. There is deliberately no retry and no Drive."""
    try:
        transport.connect()
        status, right, left = transport.verify_stop()
        mode = transport.query_oi_mode()
        validate_stop_gate(status, right, left, mode)
        counts = transport.query_encoders()
        if len(counts) != 2:
            raise RuntimeError("encoder acquisition did not return two counts")
        return int(counts[0]), int(counts[1])
    finally:
        transport.close()


@dataclass(frozen=True)
class IntervalResult:
    before_left_u16: int
    before_right_u16: int
    after_left_u16: int
    after_right_u16: int
    left_delta_ticks: int
    right_delta_ticks: int
    left_distance_m: float
    right_distance_m: float
    estimated_distance_m: float
    estimated_yaw_rad: float
    measured_distance_m: float | None
    measured_yaw_rad: float | None
    distance_error_m: float | None
    yaw_error_rad: float | None
    expected_left_sign: int | None
    expected_right_sign: int | None
    left_sign_matches: bool | None
    right_sign_matches: bool | None
    anomalous_jump: bool

    def row(self):
        return asdict(self)


def evaluate_interval(
    before_left,
    before_right,
    after_left,
    after_right,
    *,
    encoder_counts_per_rev=508.8,
    wheel_diameter_m=0.072,
    wheel_separation_m=0.235,
    max_tick_delta=2000,
    measured_distance_m=None,
    measured_yaw_rad=None,
    expected_left_sign=None,
    expected_right_sign=None,
):
    counts_per_rev = float(encoder_counts_per_rev)
    diameter = float(wheel_diameter_m)
    separation = float(wheel_separation_m)
    if not all(math.isfinite(value) and value > 0 for value in (
        counts_per_rev, diameter, separation
    )):
        raise ValueError("wheel geometry must contain finite positive values")
    if not isinstance(max_tick_delta, int) or max_tick_delta < 1:
        raise ValueError("max_tick_delta must be a positive integer")
    for expected in (expected_left_sign, expected_right_sign):
        if expected not in (None, -1, 0, 1):
            raise ValueError("expected signs must be -1, 0, 1, or None")
    left_delta = encoder_delta(after_left, before_left)
    right_delta = encoder_delta(after_right, before_right)
    metres_per_tick = math.pi * diameter / counts_per_rev
    left_distance = left_delta * metres_per_tick
    right_distance = right_delta * metres_per_tick
    distance = (left_distance + right_distance) / 2.0
    yaw = (right_distance - left_distance) / separation
    measured_distance = finite_optional(measured_distance_m, "measured_distance_m")
    measured_yaw = finite_optional(measured_yaw_rad, "measured_yaw_rad")
    return IntervalResult(
        int(before_left), int(before_right), int(after_left), int(after_right),
        left_delta, right_delta, left_distance, right_distance, distance, yaw,
        measured_distance, measured_yaw,
        None if measured_distance is None else distance - measured_distance,
        None if measured_yaw is None else yaw - measured_yaw,
        expected_left_sign, expected_right_sign,
        None if expected_left_sign is None else sign(left_delta) == expected_left_sign,
        None if expected_right_sign is None else sign(right_delta) == expected_right_sign,
        abs(left_delta) > max_tick_delta or abs(right_delta) > max_tick_delta,
    )


def require_drive_approval(session, confirmation):
    if confirmation != ATTENDED_DRIVE_APPROVAL:
        raise PermissionError("explicit attended Drive approval is required")
    if session.get("state") != "before_acquired":
        raise RuntimeError("Drive approval requires a completed before acquisition")
    session["drive_approved"] = True
    session["state"] = "drive_approved"


def require_after_gate(session, confirmation):
    if not session.get("drive_approved") or session.get("state") != "drive_approved":
        raise PermissionError("Drive was not explicitly approved for this session")
    if confirmation != PHYSICAL_STOP_CONFIRMATION:
        raise PermissionError("physical STOP confirmation is required")

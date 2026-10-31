"""Observation-only comparison of requested wheel speeds and encoder samples."""

import math
from dataclasses import asdict, dataclass

try:
    from localization_core import encoder_delta
except ModuleNotFoundError:
    from .localization_core import encoder_delta


@dataclass(frozen=True)
class MonitorEvent:
    event: str
    severity: str
    received_at_s: float
    source_timestamp_s: float | None
    data_age_s: float | None
    requested_left_mm_s: int
    requested_right_mm_s: int
    left_encoder: int | None
    right_encoder: int | None
    left_delta: int | None
    right_delta: int | None
    estimated_distance_m: float | None
    estimated_yaw_rad: float | None
    arm_generation: int | None
    motion_state: str
    detail: str = ""

    def row(self):
        return asdict(self)


class WheelMotionMonitor:
    """Keep monitoring evidence without authorizing or stopping motion."""

    def __init__(
        self,
        *,
        encoder_counts_per_rev=508.8,
        wheel_diameter_m=0.072,
        wheel_separation_m=0.235,
        command_no_motion_grace_s=0.25,
        stop_motion_grace_s=0.20,
        measurement_stale_s=0.50,
        encoder_motion_threshold_ticks=1,
        command_motion_threshold_mm_s=1,
        max_tick_delta=2000,
        started_at_s=0.0,
    ):
        positive = (
            encoder_counts_per_rev,
            wheel_diameter_m,
            wheel_separation_m,
            command_no_motion_grace_s,
            stop_motion_grace_s,
            measurement_stale_s,
        )
        if any(not math.isfinite(float(value)) or float(value) <= 0 for value in positive):
            raise ValueError("monitor dimensions and time thresholds must be positive")
        if not isinstance(encoder_motion_threshold_ticks, int) or encoder_motion_threshold_ticks < 0:
            raise ValueError("encoder_motion_threshold_ticks must be a non-negative integer")
        if not isinstance(command_motion_threshold_mm_s, int) or command_motion_threshold_mm_s < 0:
            raise ValueError("command_motion_threshold_mm_s must be a non-negative integer")
        if not isinstance(max_tick_delta, int) or max_tick_delta < 1:
            raise ValueError("max_tick_delta must be a positive integer")
        self.metres_per_tick = math.pi * float(wheel_diameter_m) / float(encoder_counts_per_rev)
        self.wheel_separation_m = float(wheel_separation_m)
        self.command_no_motion_grace_s = float(command_no_motion_grace_s)
        self.stop_motion_grace_s = float(stop_motion_grace_s)
        self.measurement_stale_s = float(measurement_stale_s)
        self.encoder_motion_threshold_ticks = encoder_motion_threshold_ticks
        self.command_motion_threshold_mm_s = command_motion_threshold_mm_s
        self.max_tick_delta = max_tick_delta
        self.started_at_s = float(started_at_s)
        self.requested_left = 0
        self.requested_right = 0
        self.command_at = None
        self.stop_at = None
        self.arm_generation = None
        self.motion_state = "stopped"
        self.previous_counts = None
        self.last_encoder_received_at = None
        self.stale_reported = False

    def observe_arm_request(self, generation, received_at_s):
        if not isinstance(generation, int) or isinstance(generation, bool) or generation < 1:
            raise ValueError("ARM generation must be a positive integer")
        self.arm_generation = generation
        self.motion_state = "arm_requested"
        return self._event("arm_requested", "info", received_at_s, detail="ACK not observed")

    def observe_command(self, left_mm_s, right_mm_s, received_at_s):
        left_mm_s, right_mm_s = int(left_mm_s), int(right_mm_s)
        self.requested_left = left_mm_s
        self.requested_right = right_mm_s
        self.command_at = float(received_at_s)
        if self._command_nonzero():
            self.stop_at = None
            self.motion_state = "drive_requested"
            event = "drive_requested"
        else:
            self.stop_at = self.command_at
            self.motion_state = "stop_requested"
            event = "stop_requested"
        return self._event(event, "info", received_at_s)

    def observe_encoder(self, sample, received_at_s):
        received_at_s = float(received_at_s)
        self.last_encoder_received_at = received_at_s
        self.stale_reported = False
        source = float(sample.timestamp)
        age = received_at_s - source
        if age < -0.1:
            self.previous_counts = None
            return self._event(
                "measurement_unavailable", "warning", received_at_s, source, age,
                sample.left_encoder, sample.right_encoder, detail="future timestamp",
            )
        if age > self.measurement_stale_s:
            self.previous_counts = None
            return self._event(
                "measurement_stale", "warning", received_at_s, source, age,
                sample.left_encoder, sample.right_encoder,
            )
        if not sample.valid or sample.communication_state != "connected":
            self.previous_counts = None
            return self._event(
                "measurement_unavailable", "warning", received_at_s, source, age,
                sample.left_encoder, sample.right_encoder,
                detail=sample.error or sample.communication_state,
            )
        counts = (sample.left_encoder, sample.right_encoder)
        if self.previous_counts is None:
            self.previous_counts = counts
            return self._event(
                "encoder_baseline", "info", received_at_s, source, age,
                *counts, detail="no motion inferred from baseline",
            )
        left_delta = encoder_delta(counts[0], self.previous_counts[0])
        right_delta = encoder_delta(counts[1], self.previous_counts[1])
        self.previous_counts = counts
        if abs(left_delta) > self.max_tick_delta or abs(right_delta) > self.max_tick_delta:
            return self._event(
                "encoder_jump_rebaseline", "warning", received_at_s, source, age,
                *counts, left_delta, right_delta,
                detail="delta exceeds max_tick_delta; not integrated",
            )
        left_m = left_delta * self.metres_per_tick
        right_m = right_delta * self.metres_per_tick
        distance = (left_m + right_m) / 2.0
        yaw = (right_m - left_m) / self.wheel_separation_m
        moving = max(abs(left_delta), abs(right_delta)) >= self.encoder_motion_threshold_ticks
        event, severity, detail = "encoder_update", "info", ""
        if self._command_nonzero() and received_at_s - self.command_at >= self.command_no_motion_grace_s and not moving:
            event, severity = "command_without_encoder_change", "warning"
        elif self.stop_at is not None and moving:
            elapsed = received_at_s - self.stop_at
            if elapsed <= self.stop_motion_grace_s:
                event, detail = "stop_coast_observed", "within configured warning grace"
            else:
                event, severity = "wheel_change_after_stop", "warning"
        return self._event(
            event, severity, received_at_s, source, age, *counts,
            left_delta, right_delta, distance, yaw, detail,
        )

    def observe_unavailable(self, received_at_s, detail):
        self.previous_counts = None
        return self._event(
            "measurement_unavailable", "warning", received_at_s, detail=str(detail)
        )

    def poll(self, now_s):
        now_s = float(now_s)
        reference = self.last_encoder_received_at
        age = now_s - (self.started_at_s if reference is None else reference)
        if age <= self.measurement_stale_s or self.stale_reported:
            return None
        self.stale_reported = True
        detail = "no encoder sample received" if reference is None else "encoder input timed out"
        return self._event("measurement_stale", "warning", now_s, data_age=age, detail=detail)

    def _command_nonzero(self):
        return max(abs(self.requested_left), abs(self.requested_right)) >= self.command_motion_threshold_mm_s

    def _event(
        self, event, severity, received_at, source=None, data_age=None,
        left=None, right=None, left_delta=None, right_delta=None,
        distance=None, yaw=None, detail="",
    ):
        return MonitorEvent(
            event, severity, float(received_at), source, data_age,
            self.requested_left, self.requested_right, left, right,
            left_delta, right_delta, distance, yaw, self.arm_generation,
            self.motion_state, detail,
        )

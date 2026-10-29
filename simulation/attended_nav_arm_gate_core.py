"""ROS-independent one-shot ARM gate for an attended Navigation trial."""

import math


ATTENDED_CONFIRMATION = "ATTENDED_FLOOR"


class FreshNavigationArmGate:
    """Return one generation only after consecutive fresh non-zero candidates."""

    def __init__(
        self,
        generation,
        confirmation,
        *,
        started_at,
        timeout_s=5.0,
        required_candidates=2,
        max_candidate_gap_s=0.25,
    ):
        if not (
            isinstance(generation, int)
            and not isinstance(generation, bool)
            and 1 <= generation <= 0xFFFFFFFF
        ):
            raise ValueError("generation must be from 1 to 4294967295")
        if confirmation != ATTENDED_CONFIRMATION:
            raise PermissionError(
                "attended ARM requires confirmation={}".format(
                    ATTENDED_CONFIRMATION
                )
            )
        for name, value in (
            ("started_at", started_at),
            ("timeout_s", timeout_s),
            ("max_candidate_gap_s", max_candidate_gap_s),
        ):
            if not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError("{} must be finite".format(name))
        if timeout_s <= 0.0 or max_candidate_gap_s <= 0.0:
            raise ValueError("timeouts must be positive")
        if not isinstance(required_candidates, int) or required_candidates < 2:
            raise ValueError("required_candidates must be an integer >= 2")
        self.generation = generation
        self.started_at = float(started_at)
        self.timeout_s = float(timeout_s)
        self.required_candidates = required_candidates
        self.max_candidate_gap_s = float(max_candidate_gap_s)
        self.candidate_count = 0
        self.last_candidate_at = None
        self.finished = False

    def check_timeout(self, now):
        if self.finished:
            return
        if now - self.started_at >= self.timeout_s:
            raise TimeoutError("no fresh Navigation candidate before ARM timeout")

    def observe(self, linear_x, angular_z, now, arm_subscription_count):
        if self.finished:
            return None
        for name, value in (
            ("linear_x", linear_x),
            ("angular_z", angular_z),
            ("now", now),
        ):
            if not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError("{} must be finite".format(name))
        self.check_timeout(now)
        if arm_subscription_count == 0:
            self.candidate_count = 0
            self.last_candidate_at = None
            return None
        if arm_subscription_count != 1:
            raise RuntimeError(
                "expected exactly one ARM subscriber, got {}".format(
                    arm_subscription_count
                )
            )
        if abs(linear_x) <= 1e-9 and abs(angular_z) <= 1e-9:
            self.candidate_count = 0
            self.last_candidate_at = None
            return None
        if (
            self.last_candidate_at is None
            or now - self.last_candidate_at > self.max_candidate_gap_s
        ):
            self.candidate_count = 1
        else:
            self.candidate_count += 1
        self.last_candidate_at = now
        if self.candidate_count < self.required_candidates:
            return None
        self.finished = True
        return self.generation


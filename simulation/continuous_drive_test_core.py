"""Pure validation for explicitly attended continuous-drive experiments."""

import math


CONTINUOUS_CONFIRMATION = "CONTINUOUS_ATTENDED_FLOOR"
MAX_SPEED_MM_S = 20
MIN_DURATION_S = 1.0
MAX_DURATION_S = 5.0
WATCHDOG_MS = 500


class ContinuousDriveTestProfile:
    def __init__(
        self,
        *,
        speed_mm_s,
        host_duration_s,
        pico_duration_ms,
        confirmation,
    ):
        for name, value in (
            ("speed_mm_s", speed_mm_s),
            ("host_duration_s", host_duration_s),
            ("pico_duration_ms", pico_duration_ms),
        ):
            if (
                not isinstance(value, (int, float))
                or isinstance(value, bool)
                or not math.isfinite(value)
            ):
                raise ValueError("{} must be finite".format(name))
        if not 0.0 < speed_mm_s <= MAX_SPEED_MM_S:
            raise ValueError("speed_mm_s must be > 0 and <= 20")
        if not MIN_DURATION_S <= host_duration_s <= MAX_DURATION_S:
            raise ValueError("host_duration_s must be 1.0..5.0")
        if int(pico_duration_ms) != pico_duration_ms:
            raise ValueError("pico_duration_ms must be an integer")
        if int(pico_duration_ms) != round(host_duration_s * 1000):
            raise ValueError("host and Pico fixed deadlines must match")
        if confirmation != CONTINUOUS_CONFIRMATION:
            raise PermissionError(
                "continuous drive requires confirmation={}".format(
                    CONTINUOUS_CONFIRMATION
                )
            )
        self.speed_mm_s = float(speed_mm_s)
        self.host_duration_s = float(host_duration_s)
        self.pico_duration_ms = int(pico_duration_ms)
        self.confirmation = confirmation

    @property
    def theoretical_distance_mm(self):
        return self.speed_mm_s * self.host_duration_s

    def as_row(self, trial_id):
        return {
            "trial_id": trial_id,
            "speed_mm_s": self.speed_mm_s,
            "host_deadline_s": self.host_duration_s,
            "pico_deadline_ms": self.pico_duration_ms,
            "watchdog_ms": WATCHDOG_MS,
            "theoretical_distance_mm": self.theoretical_distance_mm,
        }


def staged_straight_profiles():
    return tuple(
        ContinuousDriveTestProfile(
            speed_mm_s=20,
            host_duration_s=duration_s,
            pico_duration_ms=int(duration_s * 1000),
            confirmation=CONTINUOUS_CONFIRMATION,
        )
        for duration_s in (1.0, 2.0, 3.0, 5.0)
    )

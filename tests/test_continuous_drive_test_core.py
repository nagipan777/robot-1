import unittest

from simulation.continuous_drive_test_core import (
    CONTINUOUS_CONFIRMATION,
    ContinuousDriveTestProfile,
    staged_straight_profiles,
)
from simulation.roomba_tcp_transport import DriveSessionLimiter


class ContinuousDriveTestCoreTests(unittest.TestCase):
    def test_staged_profiles_are_20_40_60_100_mm(self):
        profiles = staged_straight_profiles()
        self.assertEqual(
            [profile.theoretical_distance_mm for profile in profiles],
            [20.0, 40.0, 60.0, 100.0],
        )

    def test_host_and_pico_deadlines_must_match(self):
        with self.assertRaisesRegex(ValueError, "must match"):
            ContinuousDriveTestProfile(
                speed_mm_s=20,
                host_duration_s=3.0,
                pico_duration_ms=5000,
                confirmation=CONTINUOUS_CONFIRMATION,
            )

    def test_explicit_confirmation_and_bounds_are_required(self):
        for changes in (
            {"confirmation": "ATTENDED_FLOOR"},
            {"speed_mm_s": 21},
            {"host_duration_s": 5.1, "pico_duration_ms": 5100},
        ):
            values = {
                "speed_mm_s": 20,
                "host_duration_s": 3.0,
                "pico_duration_ms": 3000,
                "confirmation": CONTINUOUS_CONFIRMATION,
            }
            values.update(changes)
            with self.subTest(changes=changes), self.assertRaises(
                (ValueError, PermissionError)
            ):
                ContinuousDriveTestProfile(**values)

    def test_continuous_limiter_does_not_slide_and_stops_at_fixed_deadline(self):
        now = [10.0]
        limiter = DriveSessionLimiter(
            3.0, lambda: now[0], max_duration_s=5.0
        )
        limiter.arm()
        self.assertEqual(limiter.observe_nonzero(), "started")
        for offset in (0.1, 0.5, 1.0, 2.9):
            now[0] = 10.0 + offset
            self.assertEqual(limiter.observe_nonzero(), "allowed")
            self.assertEqual(limiter.deadline, 13.0)
        now[0] = 13.0
        self.assertEqual(limiter.observe_nonzero(), "expired")
        self.assertEqual(limiter.observe_nonzero(), "latched")

    def test_proven_mode_still_rejects_more_than_one_second(self):
        with self.assertRaisesRegex(ValueError, "<= 1.0 seconds"):
            DriveSessionLimiter(1.001)


if __name__ == "__main__":
    unittest.main()

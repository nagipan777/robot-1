import math
import unittest

from differential_drive import twist_to_wheel_speeds


class DifferentialDriveTests(unittest.TestCase):
    def test_stop(self):
        self.assertEqual(twist_to_wheel_speeds(0.0, 0.0), (0, 0))

    def test_straight_forward(self):
        self.assertEqual(twist_to_wheel_speeds(0.03, 0.0), (30, 30))

    def test_straight_reverse(self):
        self.assertEqual(twist_to_wheel_speeds(-0.025, 0.0), (-25, -25))

    def test_rotation_in_place(self):
        self.assertEqual(twist_to_wheel_speeds(0.0, 0.4), (-47, 47))
        self.assertEqual(twist_to_wheel_speeds(0.0, -0.4), (47, -47))

    def test_curved_motion(self):
        self.assertEqual(twist_to_wheel_speeds(0.02, 0.1), (8, 32))

    def test_custom_track_width(self):
        self.assertEqual(
            twist_to_wheel_speeds(0.0, 0.2, track_width_m=0.5),
            (-50, 50),
        )

    def test_speed_limit_scales_both_wheels_proportionally(self):
        left, right = twist_to_wheel_speeds(0.1, 0.2)
        self.assertEqual((left, right), (31, 50))
        raw_ratio = 76.5 / 123.5
        self.assertAlmostEqual(left / right, raw_ratio, delta=0.01)

    def test_configured_speed_limit_matches_driver_units(self):
        self.assertEqual(
            twist_to_wheel_speeds(0.2, 0.0, max_wheel_speed_mm_s=40),
            (40, 40),
        )

    def test_non_finite_commands_are_rejected(self):
        for linear, angular in (
            (math.nan, 0.0),
            (math.inf, 0.0),
            (-math.inf, 0.0),
            (0.0, math.nan),
            (0.0, math.inf),
        ):
            with self.subTest(linear=linear, angular=angular):
                with self.assertRaises(ValueError):
                    twist_to_wheel_speeds(linear, angular)

    def test_invalid_track_width_is_rejected(self):
        for width in (0.0, -0.1, math.nan, math.inf):
            with self.subTest(width=width):
                with self.assertRaises(ValueError):
                    twist_to_wheel_speeds(0.0, 0.0, track_width_m=width)

    def test_invalid_value_types_are_rejected(self):
        with self.assertRaises(TypeError):
            twist_to_wheel_speeds("fast", 0.0)
        with self.assertRaises(TypeError):
            twist_to_wheel_speeds(0.0, False)

    def test_invalid_speed_limit_is_rejected(self):
        for limit in (0, -1, 501, math.nan, math.inf):
            with self.subTest(limit=limit):
                with self.assertRaises(ValueError):
                    twist_to_wheel_speeds(
                        0.0,
                        0.0,
                        max_wheel_speed_mm_s=limit,
                    )


if __name__ == "__main__":
    unittest.main()

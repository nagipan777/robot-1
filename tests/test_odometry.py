import math
import unittest

from odometry import ENCODER_MODULO, Odometry, Pose2D


class OdometryTests(unittest.TestCase):
    def setUp(self):
        # These values make one encoder tick equal exactly one metre.
        self.odometry = Odometry(
            wheel_diameter=1.0 / math.pi,
            wheel_base=2.0,
            ticks_per_rev=1.0,
        )

    def test_first_sample_sets_baseline_without_motion(self):
        pose = self.odometry.update(100, 200)
        self.assertEqual(pose, Pose2D())

    def test_straight_forward_and_reverse(self):
        self.odometry.update(100, 100)
        pose = self.odometry.update(110, 110)
        self.assertAlmostEqual(pose.x, 10.0)
        self.assertAlmostEqual(pose.y, 0.0)
        self.assertAlmostEqual(pose.theta, 0.0)

        pose = self.odometry.update(105, 105)
        self.assertAlmostEqual(pose.x, 5.0)
        self.assertAlmostEqual(pose.y, 0.0)

    def test_rotation_in_place(self):
        self.odometry.update(100, 100)
        pose = self.odometry.update(99, 101)
        self.assertAlmostEqual(pose.x, 0.0)
        self.assertAlmostEqual(pose.y, 0.0)
        self.assertAlmostEqual(pose.theta, 1.0)

    def test_curved_motion_uses_midpoint_heading(self):
        self.odometry.update(100, 100)
        pose = self.odometry.update(102, 104)
        self.assertAlmostEqual(pose.x, 3.0 * math.cos(0.5))
        self.assertAlmostEqual(pose.y, 3.0 * math.sin(0.5))
        self.assertAlmostEqual(pose.theta, 1.0)

    def test_encoder_wraparound_in_both_directions(self):
        self.assertEqual(Odometry.tick_diff(2, 65534), 4)
        self.assertEqual(Odometry.tick_diff(65534, 2), -4)

    def test_heading_is_normalized(self):
        self.odometry.reset(Pose2D(theta=math.pi - 0.1))
        self.odometry.update(100, 100)
        pose = self.odometry.update(99, 101)
        self.assertGreaterEqual(pose.theta, -math.pi)
        self.assertLessEqual(pose.theta, math.pi)
        self.assertAlmostEqual(pose.theta, -math.pi + 0.9)

    def test_reset_requires_new_baseline(self):
        self.odometry.update(100, 100)
        self.odometry.update(110, 110)
        self.odometry.reset(Pose2D(1.0, 2.0, 0.5))
        pose = self.odometry.update(500, 600)
        self.assertEqual(pose, Pose2D(1.0, 2.0, 0.5))

    def test_invalid_encoder_value_is_rejected(self):
        for invalid in (-1, ENCODER_MODULO):
            with self.subTest(invalid=invalid):
                with self.assertRaises(ValueError):
                    self.odometry.update(invalid, 0)
        with self.assertRaises(TypeError):
            self.odometry.update(1.5, 0)


if __name__ == "__main__":
    unittest.main()

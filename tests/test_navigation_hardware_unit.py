import tempfile
import unittest
from pathlib import Path

from simulation.navigation_hardware_unit_core import evaluate_navigation_case
from simulation.run_navigation_hardware_phase0 import run


class NavigationHardwareUnitTests(unittest.TestCase):
    def test_forward_approach_is_scaled_to_twenty_mm_s(self):
        result = evaluate_navigation_case("forward", target=(1.0, 0.0), mode="approach")
        self.assertEqual((result.left_mm_s, result.right_mm_s), (20, 20))
        self.assertAlmostEqual(result.linear_x_m_s, 0.08)

    def test_left_and_right_rotation_signs(self):
        left = evaluate_navigation_case("left", target=(0.0, 1.0), mode="turn_only")
        right = evaluate_navigation_case("right", target=(0.0, -1.0), mode="turn_only")
        self.assertEqual((left.left_mm_s, left.right_mm_s), (-20, 20))
        self.assertEqual((right.left_mm_s, right.right_mm_s), (20, -20))
        self.assertEqual(left.expected_motion, "left_rotation")
        self.assertEqual(right.expected_motion, "right_rotation")

    def test_forty_five_degree_commands_are_angularly_saturated(self):
        left = evaluate_navigation_case("left45", target=(1.0, 1.0), mode="turn_only")
        right = evaluate_navigation_case("right45", target=(1.0, -1.0), mode="turn_only")
        self.assertAlmostEqual(left.angular_z_rad_s, 0.8)
        self.assertAlmostEqual(right.angular_z_rad_s, -0.8)
        self.assertEqual((left.left_mm_s, left.right_mm_s), (-20, 20))
        self.assertEqual((right.left_mm_s, right.right_mm_s), (20, -20))

    def test_reached_missing_stale_and_stopped_target_are_zero(self):
        results = (
            evaluate_navigation_case("reached", target=(0.1, 0.0)),
            evaluate_navigation_case("pose", pose_fresh=False),
            evaluate_navigation_case("target", target_fresh=False),
            evaluate_navigation_case("old", stopped_target_latched=True),
        )
        self.assertTrue(all(result.stop for result in results))
        self.assertTrue(all((result.left_mm_s, result.right_mm_s) == (0, 0) for result in results))

    def test_angle_normalization_uses_shortest_turn(self):
        result = evaluate_navigation_case(
            "wrap", robot_pose=(0.0, 0.0, 3.10), target=(-1.0, -0.01), mode="turn_only"
        )
        self.assertGreater(result.angle_error_rad, 0.0)
        self.assertLess(abs(result.angle_error_rad), 0.1)

    def test_phase0_artifacts_pass_without_arm_or_drive(self):
        with tempfile.TemporaryDirectory() as directory:
            payload = run(Path(directory) / "result.json", Path(directory) / "result.csv")
            self.assertTrue(payload["passed"])
            self.assertFalse(payload["physical_hardware_used"])
            self.assertFalse(payload["arm_sent"])
            self.assertFalse(payload["drive_sent"])


if __name__ == "__main__":
    unittest.main()

import pathlib
import unittest

from simulation.attended_nav_arm_gate_core import (
    ATTENDED_CONFIRMATION,
    FreshNavigationArmGate,
)


ROOT = pathlib.Path(__file__).resolve().parents[1]


class FreshNavigationArmGateTests(unittest.TestCase):
    def gate(self, **kwargs):
        return FreshNavigationArmGate(
            123,
            ATTENDED_CONFIRMATION,
            started_at=10.0,
            **kwargs,
        )

    def test_requires_confirmation_and_explicit_generation(self):
        with self.assertRaises(PermissionError):
            FreshNavigationArmGate(123, "", started_at=0.0)
        for generation in (0, -1, 0x100000000, True):
            with self.subTest(generation=generation):
                with self.assertRaises(ValueError):
                    FreshNavigationArmGate(
                        generation, ATTENDED_CONFIRMATION, started_at=0.0
                    )

    def test_two_fresh_candidates_publish_once(self):
        gate = self.gate()
        self.assertIsNone(gate.observe(0.08, 0.0, 10.10, 1))
        self.assertEqual(gate.observe(0.08, 0.0, 10.20, 1), 123)
        self.assertIsNone(gate.observe(0.08, 0.0, 10.21, 1))

    def test_zero_and_gap_reset_sequence(self):
        gate = self.gate()
        self.assertIsNone(gate.observe(0.08, 0.0, 10.10, 1))
        self.assertIsNone(gate.observe(0.0, 0.0, 10.15, 1))
        self.assertIsNone(gate.observe(0.08, 0.0, 10.20, 1))
        self.assertIsNone(gate.observe(0.08, 0.0, 10.50, 1))
        self.assertEqual(gate.observe(0.08, 0.0, 10.60, 1), 123)

    def test_timeout_fails_without_arm(self):
        gate = self.gate(timeout_s=1.0)
        with self.assertRaises(TimeoutError):
            gate.check_timeout(11.0)
        self.assertFalse(gate.finished)

    def test_waits_for_one_arm_subscriber_and_rejects_duplicates(self):
        gate = self.gate()
        self.assertIsNone(gate.observe(0.08, 0.0, 10.1, 0))
        self.assertEqual(gate.candidate_count, 0)
        with self.assertRaises(RuntimeError):
            gate.observe(0.08, 0.0, 10.2, 2)

    def test_rejects_non_finite_input(self):
        with self.assertRaises(ValueError):
            self.gate().observe(float("nan"), 0.0, 10.1, 1)

    def test_ros_node_has_no_drive_publisher_or_retry(self):
        source = (ROOT / "simulation" / "attended_nav_arm_gate.py").read_text()
        self.assertIn('"/nav/cmd_vel_unstamped"', source)
        self.assertIn('"/roomba/arm_generation"', source)
        self.assertNotIn('"/roomba/cmd_vel_unstamped"', source)
        self.assertEqual(source.count("self.arm_publisher.publish(arm)"), 1)
        self.assertNotIn("create_publisher(Twist", source)


if __name__ == "__main__":
    unittest.main()


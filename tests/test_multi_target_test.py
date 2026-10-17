import json
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class MultiTargetTestConfigurationTests(unittest.TestCase):
    def test_two_distinct_targets_and_safety_limits(self):
        config = json.loads(
            (ROOT / "simulation" / "multi_target_test.json").read_text()
        )
        targets = config["targets_odom_m"]
        self.assertEqual(len(targets), 2)
        self.assertNotEqual(
            (targets[0]["x"], targets[0]["y"]),
            (targets[1]["x"], targets[1]["y"]),
        )
        self.assertGreaterEqual(config["post_arrival_hold_s"], 2.0)
        self.assertLessEqual(config["max_linear_speed_m_s"], 0.08)
        self.assertGreaterEqual(config["minimum_static_obstacle_clearance_m"], 1.0)

    def test_test_is_explicit_and_uses_existing_target_path(self):
        source = (ROOT / "simulation" / "multi_target_test.py").read_text()
        script = (ROOT / "simulation" / "sim.sh").read_text()
        supervisor = (ROOT / "simulation" / "supervisord.conf").read_text()
        self.assertIn('"/litter/target_odom"', source)
        self.assertIn('"/litter/target"', source)
        self.assertIn("multi-target-test)", script)
        self.assertIn("supervisorctl stop webcam-bridge", script)
        self.assertNotIn("multi-target-navigation-test", supervisor)

    def test_same_target_hold_and_new_target_release_are_checked(self):
        source = (ROOT / "simulation" / "multi_target_test.py").read_text()
        self.assertIn("same-coordinate republish released latch", source)
        self.assertIn("did not release the arrival latch", source)
        self.assertIn("post_reach_nonzero_commands", source)
        self.assertIn("stale_nonzero_commands_before_latch_release", source)
        self.assertIn("arrival_epsilon=0.001", source)


if __name__ == "__main__":
    unittest.main()

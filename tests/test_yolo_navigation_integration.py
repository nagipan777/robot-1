import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class YoloNavigationIntegrationTests(unittest.TestCase):
    def test_runner_is_simulation_only_and_restores_safety_flags(self):
        source = (ROOT / "simulation/yolo_navigation_test.py").read_text()
        script = (ROOT / "simulation/sim.sh").read_text()
        self.assertIn('"control_enabled": False, "calibration_ready": False', source)
        self.assertIn('SetParameters, "/vision_turn_controller/set_parameters"', source)
        calibration = source.index('"calibration_ready": True')
        ready_wait = source.index("ready = wait_until")
        self.assertLess(calibration, ready_wait)
        self.assertIn('"/cmd_vel_unstamped"', source)
        self.assertNotIn("/roomba/cmd_vel_unstamped", source)
        self.assertIn("yolo-navigation-test)", script)
        self.assertIn("SPAWN_DOCK", script)

    def test_runner_checks_arrival_latch_and_speed_limit(self):
        source = (ROOT / "simulation/yolo_navigation_test.py").read_text()
        self.assertIn('"max_linear_speed": 0.08', source)
        self.assertIn("post_reach_nonzero_commands", source)
        self.assertIn("final_position_error_m", source)
        self.assertIn("hazard_count", source)


if __name__ == "__main__":
    unittest.main()

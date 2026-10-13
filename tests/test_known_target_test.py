import json
import unittest
from pathlib import Path


class KnownTargetConfigurationTests(unittest.TestCase):
    def test_defaults_remain_within_simulation_safety_limit(self):
        root = Path(__file__).resolve().parents[1]
        config = json.loads((root / "simulation/known_target_test.json").read_text())
        self.assertLessEqual(config["max_linear_speed_m_s"], 0.08)
        self.assertGreater(config["publish_rate_hz"], 1.0 / config["input_timeout_s"])

    def test_normal_start_does_not_run_known_target_test(self):
        root = Path(__file__).resolve().parents[1]
        compose = (root / "simulation/compose.yaml").read_text()
        supervisor = (root / "simulation/supervisord.conf").read_text()
        self.assertNotIn("known-target-navigation-test", compose)
        self.assertNotIn("known-target-navigation-test", supervisor)

    def test_navigation_test_start_is_explicit_and_uses_sim_config(self):
        root = Path(__file__).resolve().parents[1]
        script = (root / "simulation/sim.sh").read_text()
        self.assertIn("nav-test-up)", script)
        self.assertIn("VISION_CONTROLLER_CONFIG=/etc/vision-controller.sim-camera.yaml", script)
        self.assertIn("ROBOT_Y=-0.75", script)
        self.assertIn("SPAWN_DOCK=false", script)

    def test_normal_launch_keeps_original_pose_and_dock_defaults(self):
        root = Path(__file__).resolve().parents[1]
        compose = (root / "simulation/compose.yaml").read_text()
        launch = (
            root
            / "simulation/create3_sim/irobot_create_gz/irobot_create_gz_bringup/launch/create3_gz.launch.py"
        ).read_text()
        self.assertIn('${ROBOT_Y:-0.0}', compose)
        self.assertIn('${SPAWN_DOCK:-true}', compose)
        self.assertIn("DeclareLaunchArgument('spawn_dock', default_value='true'", launch)

    def test_arrival_epsilon_is_simulation_only_and_reversible(self):
        root = Path(__file__).resolve().parents[1]
        normal = (root / "simulation/vision_controller.yaml").read_text()
        simulated = (root / "simulation/vision_controller.sim_camera.yaml").read_text()
        controller = (root / "simulation/vision_turn_controller.py").read_text()
        self.assertIn("arrival_epsilon: 0.0", normal)
        self.assertIn("arrival_epsilon: 0.001", simulated)
        self.assertIn('declare_parameter("arrival_epsilon", 0.0)', controller)
        self.assertIn("distance <= arrival_distance + arrival_epsilon", controller)
        self.assertIn("self.target_reached", controller)


if __name__ == "__main__":
    unittest.main()

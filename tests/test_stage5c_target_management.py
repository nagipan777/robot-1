import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class Stage5CTargetManagementConfigurationTests(unittest.TestCase):
    def test_normal_configuration_keeps_new_gate_disabled(self):
        controller = (ROOT / "simulation/vision_turn_controller.py").read_text()
        normal = (ROOT / "simulation/vision_controller.yaml").read_text()
        self.assertIn(
            'declare_parameter("safe_target_management_enabled", False)',
            controller,
        )
        self.assertNotIn("safe_target_management_enabled: true", normal)

    def test_integration_configuration_enables_all_safety_gates(self):
        config = (
            ROOT / "simulation/vision_controller.integration.yaml"
        ).read_text()
        self.assertRegex(
            config,
            r"(?m)^\s*safe_target_management_enabled:\s*true$",
        )
        self.assertRegex(config, r"(?m)^\s*target_change_distance:\s*0\.20$")
        self.assertRegex(config, r"(?m)^\s*min_detection_confidence:\s*0\.25$")
        self.assertRegex(config, r"(?m)^\s*max_detection_age:\s*3\.0$")
        self.assertRegex(config, r"(?m)^\s*input_timeout:\s*20\.0$")
        self.assertRegex(config, r"(?m)^\s*control_enabled:\s*false$")
        self.assertRegex(config, r"(?m)^\s*calibration_ready:\s*false$")

    def test_container_installs_target_manager_dependency(self):
        dockerfile = (ROOT / "simulation/Dockerfile").read_text()
        self.assertIn(
            "COPY target_manager.py /usr/local/bin/target_manager.py",
            dockerfile,
        )

    def test_detection_loss_blocks_resume_only_while_control_is_enabled(self):
        controller = (ROOT / "simulation/vision_turn_controller.py").read_text()
        self.assertIn(
            "preserve_safe_state = safe_management and control_enabled",
            controller,
        )


if __name__ == "__main__":
    unittest.main()

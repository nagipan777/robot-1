import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]

class CmdVelOutputAdapterTests(unittest.TestCase):
    def test_modes_are_explicit(self):
        source = (ROOT / "simulation/cmd_vel_output_adapter.py").read_text()
        self.assertIn('VALID_MODES = ("gazebo", "roomba", "dry_run")', source)

    def test_rejects_non_finite_command(self):
        source = (ROOT / "simulation/cmd_vel_output_adapter.py").read_text()
        self.assertIn("math.isfinite(value)", source)
        self.assertIn('publish_stop("non-finite command")', source)

    def test_default_config_never_targets_physical_topic(self):
        config = (ROOT / "simulation/cmd_vel_output_adapter.yaml").read_text()
        self.assertRegex(config, r"(?m)^\s*mode:\s*dry_run$")
        self.assertIn("dry_run_topic: /roomba/cmd_vel_dry_run", config)

    def test_adapter_has_timeout_and_shutdown_stops(self):
        source = (ROOT / "simulation/cmd_vel_output_adapter.py").read_text()
        self.assertIn('publish_stop("navigation command timeout")', source)
        self.assertIn('publish_stop("shutdown")', source)


if __name__ == "__main__":
    unittest.main()

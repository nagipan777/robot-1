import unittest
from pathlib import Path


class OscillationDiagnosticsTests(unittest.TestCase):
    def test_diagnostics_are_read_only_and_cover_control_path(self):
        root = Path(__file__).resolve().parents[1]
        source = (root / "simulation/oscillation_diagnostics.py").read_text()
        for topic in (
            "/cmd_vel_unstamped",
            "/cmd_vel",
            "/diffdrive_controller/cmd_vel",
            "/odom",
            "/hazard_detection",
        ):
            self.assertIn(topic, source)
        self.assertNotIn("create_publisher", source)
        self.assertNotIn("set_parameters", source)


if __name__ == "__main__":
    unittest.main()

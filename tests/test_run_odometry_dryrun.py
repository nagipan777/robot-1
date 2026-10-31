import json
import tempfile
import unittest
from pathlib import Path

from simulation.run_odometry_dryrun import run


class RunOdometryDryRunTests(unittest.TestCase):
    def test_one_command_outputs_csv_metrics_and_svg(self):
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as directory:
            result = run(
                root / "docs" / "odometry_dryrun_inputs" / "left_turn.csv",
                directory,
            )
            output = Path(directory)
            for name in (
                "odometry.csv", "aruco_synthetic.csv", "comparison.csv",
                "summary.json", "trajectory.svg",
            ):
                self.assertTrue((output / name).is_file(), name)
            summary = json.loads((output / "summary.json").read_text())
            self.assertEqual(summary["data_kind"], "synthetic")
            self.assertFalse(summary["real_uart_used"])
            self.assertFalse(summary["nonzero_drive_sent"])
            self.assertAlmostEqual(summary["position_rmse_m"], 0.0, places=12)
            self.assertAlmostEqual(summary["yaw_rmse_rad"], 0.0, places=12)
            self.assertEqual(result["matched_samples"], 3)

    def test_abnormal_input_is_logged_without_pose_jump(self):
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as directory:
            run(
                root / "docs" / "odometry_dryrun_inputs" / "abnormal_jump.csv",
                directory,
            )
            rows = (Path(directory) / "odometry.csv").read_text().splitlines()
            self.assertIn("encoder jump exceeds max_tick_delta", rows[-1])
            self.assertIn(",False,connected,", rows[-1])


if __name__ == "__main__":
    unittest.main()

import json
import math
import subprocess
import tempfile
import unittest
from unittest import mock
from pathlib import Path

from simulation.replay_create3_trajectory import (
    GazeboPoseTimeout,
    interpolate,
    load_passed_trajectory,
    pose_request,
    replay,
    set_pose,
)


class Create3TrajectoryReplayTests(unittest.TestCase):
    def write_result(self, payload):
        temporary = tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False)
        json.dump(payload, temporary)
        temporary.close()
        self.addCleanup(Path(temporary.name).unlink, missing_ok=True)
        return temporary.name

    def test_loads_only_passed_finite_monotonic_trajectory(self):
        path = self.write_result(
            {
                "status": "passed",
                "trial_id": "test",
                "trajectory": [
                    {"time_s": 1, "x": 0, "y": 0, "theta": 0},
                    {"time_s": 2, "x": 1, "y": 2, "theta": 0.5},
                ],
            }
        )
        payload, samples = load_passed_trajectory(path)
        self.assertEqual(payload["trial_id"], "test")
        self.assertEqual(samples[-1], (2.0, 1.0, 2.0, 0.5))

        for invalid in (
            {"status": "failed", "trajectory": []},
            {"status": "passed", "trajectory": [{"time_s": 1}]},
            {
                "status": "passed",
                "trajectory": [
                    {"time_s": 1, "x": 0, "y": 0, "theta": 0},
                    {"time_s": 1, "x": 1, "y": 1, "theta": 1},
                ],
            },
        ):
            with self.subTest(invalid=invalid):
                with self.assertRaises(ValueError):
                    load_passed_trajectory(self.write_result(invalid))

    def test_interpolates_position_and_shortest_yaw(self):
        samples = [(0.0, 0.0, 0.0, 3.0), (2.0, 2.0, 4.0, -3.0)]
        x, y, yaw = interpolate(samples, 1.0)
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 2.0)
        self.assertAlmostEqual(abs(yaw), math.pi)

    def test_dry_run_preserves_timing_without_gazebo_or_velocity(self):
        samples = [(5.0, 0.0, 0.0, 0.0), (15.0, 1.0, 0.0, 0.0)]
        published = []
        original, playback, frames = replay(
            samples,
            "room_camera",
            "create3",
            speed=10.0,
            fps=10.0,
            dry_run=True,
            pose_callback=published.append,
        )
        self.assertEqual(original, 10.0)
        self.assertEqual(playback, 1.0)
        self.assertEqual(frames, 11)
        self.assertEqual(published[0], (0.0, 0.0, 0.0))
        self.assertEqual(published[-1], (1.0, 0.0, 0.0))
        request = pose_request("create3", 1.0, 2.0, math.pi / 2)
        self.assertIn('name: "create3"', request)
        self.assertIn("position: {x: 1.000000000, y: 2.000000000", request)

    def test_launcher_is_camera_world_only_and_has_no_physical_output(self):
        root = Path(__file__).resolve().parents[1]
        source = (root / "simulation" / "sim.sh").read_text()
        start = source.index("  replay-create3-trajectory)")
        case = source[start : source.index("  ros-status)", start)]
        self.assertIn('[ "$SIM_WORLD" = room_camera ]', case)
        self.assertIn("control_enabled false", case)
        self.assertIn("calibration_ready false", case)
        self.assertIn("physical Roomba command topic exists", case)
        self.assertIn("ros2 topic pub --once /cmd_vel_unstamped", case)
        self.assertIn("supervisorctl stop presentation-visualizer", case)
        self.assertIn("supervisorctl start presentation-visualizer", case)
        self.assertNotIn("roomba-bridge-tcp", case)
        self.assertNotIn("/roomba/cmd_vel_unstamped geometry_msgs", case)

    @mock.patch("simulation.replay_create3_trajectory.subprocess.run")
    def test_set_pose_reports_process_timeout(self, run):
        run.side_effect = subprocess.TimeoutExpired(["gz", "service"], 2.0)
        with self.assertRaisesRegex(GazeboPoseTimeout, "timed out after 2.0s"):
            set_pose("room_camera", "create3", (0.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()

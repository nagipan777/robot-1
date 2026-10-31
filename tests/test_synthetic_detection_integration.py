import math
import pathlib
import re
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class SyntheticDetectionIntegrationTests(unittest.TestCase):
    def test_dedicated_launch_is_camera_world_dockless_and_stopped(self):
        script = (ROOT / "simulation" / "sim.sh").read_text()
        branch = script.split("vision-integration-up)", 1)[1].split(";;", 1)[0]
        self.assertIn("SIM_WORLD=room_camera", branch)
        self.assertIn("ROBOT_X=0.0 ROBOT_Y=0.0 ROBOT_YAW=0.0 SPAWN_DOCK=false", branch)
        self.assertIn("colima -p create3 start --cpu 4 --memory 6", branch)
        self.assertIn("/world/room_camera/control", branch)
        self.assertIn('pause: false', branch)
        self.assertNotIn("roomba-tcp", branch)

        config = (
            ROOT / "simulation" / "vision_controller.integration.yaml"
        ).read_text()
        self.assertRegex(config, r"(?m)^\s*control_enabled:\s*false$")
        self.assertRegex(config, r"(?m)^\s*calibration_ready:\s*false$")
        self.assertRegex(config, r"(?m)^\s*max_detection_age:\s*3\.0$")

    def test_existing_launch_commands_are_unchanged_in_scope(self):
        script = (ROOT / "simulation" / "sim.sh").read_text()
        self.assertIn('SIM_WORLD=room "${compose[@]}" up -d --build', script)
        self.assertIn(
            "SIM_WORLD=room_camera VISION_CONTROLLER_CONFIG=/etc/vision-controller.sim-camera.yaml",
            script,
        )
        self.assertIn("ROBOT_Y=-0.75 ROBOT_YAW=0.0 SPAWN_DOCK=false", script)

    def test_known_pixel_maps_to_expected_floor_coordinate(self):
        config = (
            ROOT / "simulation" / "vision_controller.integration.yaml"
        ).read_text()
        match = re.search(r"image_to_odom_homography:\s*\[([^]]+)]", config)
        self.assertIsNotNone(match)
        h = [float(value) for value in match.group(1).split(",")]
        pixel_x, pixel_y = 279.4, 239.5
        denominator = h[6] * pixel_x + h[7] * pixel_y + h[8]
        x = (h[0] * pixel_x + h[1] * pixel_y + h[2]) / denominator
        y = (h[3] * pixel_x + h[4] * pixel_y + h[5]) / denominator
        self.assertLess(math.hypot(x, y - 0.5), 0.01)

    def test_runner_uses_http_and_never_publishes_commands(self):
        source = (
            ROOT / "simulation" / "synthetic_detection_test.py"
        ).read_text()
        self.assertIn("http://127.0.0.1:8765/detections", source)
        self.assertIn('"test_data": True', source)
        self.assertNotIn("create_publisher", source)
        self.assertIn('"/roomba/cmd_vel_unstamped"', source)
        self.assertIn("non-zero /cmd_vel_unstamped was observed", source)
        self.assertIn("set_calibration_ready(node, executor, True)", source)
        self.assertIn("set_calibration_ready(node, executor, False)", source)

    def test_stale_detection_guard_is_opt_in(self):
        controller = (
            ROOT / "simulation" / "vision_turn_controller.py"
        ).read_text()
        normal = (ROOT / "simulation" / "vision_controller.sim_camera.yaml").read_text()
        integration = (
            ROOT / "simulation" / "vision_controller.integration.yaml"
        ).read_text()
        self.assertIn('declare_parameter("max_detection_age", 0.0)', controller)
        self.assertNotIn("max_detection_age", normal)
        self.assertIn("max_detection_age: 3.0", integration)


if __name__ == "__main__":
    unittest.main()

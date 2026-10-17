import math
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


CAMERA_WORLD = Path(__file__).resolve().parents[1] / "simulation/worlds/room_camera.sdf"
REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


class FixedCameraWorldTests(unittest.TestCase):
    def setUp(self):
        self.root = ET.parse(CAMERA_WORLD).getroot()
        self.sensor = self.root.find(".//model[@name='fixed_overhead_camera']//sensor")
        self.assertIsNotNone(self.sensor)

    def test_fixed_rgb_camera_configuration(self):
        self.assertEqual(self.sensor.get("type"), "camera")
        self.assertEqual(self.sensor.findtext("topic"), "/sim/fixed_camera/image_raw")
        self.assertEqual(self.sensor.findtext("camera/image/format"), "R8G8B8")
        self.assertEqual(int(self.sensor.findtext("camera/image/width")), 640)
        self.assertEqual(int(self.sensor.findtext("camera/image/height")), 480)

    def test_camera_field_of_view_covers_six_by_four_meter_floor(self):
        camera_model = self.root.find(".//model[@name='fixed_overhead_camera']")
        camera_height = float(camera_model.findtext("pose").split()[2])
        horizontal_fov = float(self.sensor.findtext("camera/horizontal_fov"))
        width = int(self.sensor.findtext("camera/image/width"))
        height = int(self.sensor.findtext("camera/image/height"))
        horizontal_coverage = 2.0 * camera_height * math.tan(horizontal_fov / 2.0)
        vertical_fov = 2.0 * math.atan(math.tan(horizontal_fov / 2.0) * height / width)
        vertical_coverage = 2.0 * camera_height * math.tan(vertical_fov / 2.0)
        self.assertGreaterEqual(horizontal_coverage, 6.0)
        self.assertGreaterEqual(vertical_coverage, 4.0)

    def test_mjpeg_is_localhost_only_and_camera_up_only(self):
        compose = (REPOSITORY_ROOT / "simulation/compose.yaml").read_text()
        start_script = (REPOSITORY_ROOT / "simulation/start-camera-mjpeg.sh").read_text()
        supervisor = (REPOSITORY_ROOT / "simulation/supervisord.conf").read_text()
        self.assertIn('"127.0.0.1:8766:8766"', compose)
        self.assertIn('SIM_WORLD:-room}', start_script)
        self.assertIn('!= "room_camera"', start_script)
        self.assertIn("start-camera-mjpeg", supervisor)
        self.assertIn("start-camera-activation", supervisor)

    def test_demo_litter_is_visual_only(self):
        model_path = REPOSITORY_ROOT / "simulation/models/demo_litter/model.sdf"
        model = ET.parse(model_path).getroot()
        self.assertIsNotNone(model.find(".//model[@name='demo_litter']//visual"))
        self.assertIsNone(model.find(".//model[@name='demo_litter']//collision"))

    def test_camera_world_uses_separate_controller_config(self):
        start_script = (
            REPOSITORY_ROOT / "simulation/start-vision-controller.sh"
        ).read_text()
        sim_config = (
            REPOSITORY_ROOT / "simulation/vision_controller.sim_camera.yaml"
        ).read_text()
        self.assertIn("vision-controller.sim-camera.yaml", start_script)
        self.assertIn("control_enabled: false", sim_config)
        self.assertIn("calibration_ready: false", sim_config)


if __name__ == "__main__":
    unittest.main()

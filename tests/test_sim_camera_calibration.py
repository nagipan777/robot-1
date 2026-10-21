import tempfile
import unittest
from pathlib import Path

import numpy as np

from calibrate_sim_camera import (
    compute_homography,
    errors_for,
    write_sim_config,
)
from collect_sim_camera_points import find_marker_center


class SimCameraCalibrationTests(unittest.TestCase):
    def test_four_points_transform_independent_validation_point(self):
        calibration = [
            [100, 100, 2, 1],
            [500, 100, 2, -1],
            [500, 400, -2, -1],
            [100, 400, -2, 1],
        ]
        homography = compute_homography(calibration)
        _projected, errors = errors_for([[300, 250, 0, 0]], homography)
        self.assertLess(errors[0], 1e-8)

    def test_requires_at_least_four_points(self):
        with self.assertRaises(ValueError):
            compute_homography([[0, 0, 0, 0]] * 3)

    def test_sim_config_save_preserves_both_safety_flags(self):
        source = """control_enabled: true
calibration_ready: true
image_to_odom_homography: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
calibration_image_width: 0
calibration_image_height: 0
"""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "sim.yaml"
            path.write_text(source)
            write_sim_config(path, np.eye(3), 640, 480)
            saved = path.read_text()
        self.assertIn("control_enabled: false", saved)
        self.assertIn("calibration_ready: false", saved)
        self.assertIn("calibration_image_width: 640", saved)
        self.assertIn("calibration_image_height: 480", saved)

    def test_magenta_marker_center_is_measured_from_image(self):
        image = np.zeros((100, 120, 3), dtype=np.uint8)
        image[40:60, 70:90] = (255, 0, 255)
        x, y = find_marker_center(image)
        self.assertAlmostEqual(x, 79.5)
        self.assertAlmostEqual(y, 49.5)


if __name__ == "__main__":
    unittest.main()

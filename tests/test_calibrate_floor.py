import unittest

import numpy as np

from calibrate_floor import compute_homography, transform


class FloorCalibrationTests(unittest.TestCase):
    def test_independent_center_point(self):
        pixels = np.float32([[100, 100], [500, 100], [500, 400], [100, 400]])
        floor = np.float32([[2.5, 1.0], [2.5, -1.0], [0.5, -1.0], [0.5, 1.0]])
        homography = compute_homography(pixels, floor)
        actual = transform([[300, 250]], homography)[0]
        np.testing.assert_allclose(actual, [1.5, 0.0], atol=1e-6)

    def test_rejects_crossed_corner_order(self):
        pixels = np.float32([[100, 100], [500, 400], [500, 100], [100, 400]])
        floor = np.float32([[2.5, 1.0], [2.5, -1.0], [0.5, -1.0], [0.5, 1.0]])
        with self.assertRaisesRegex(ValueError, "凸四角形"):
            compute_homography(pixels, floor)


if __name__ == "__main__":
    unittest.main()

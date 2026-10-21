import unittest

import numpy as np

from compare_sim_litter import box_iou, difference_region


class SimLitterComparisonTests(unittest.TestCase):
    def test_box_iou(self):
        self.assertAlmostEqual(box_iou([0, 0, 10, 10], [5, 5, 15, 15]), 25 / 175)
        self.assertEqual(box_iou([0, 0, 1, 1], [2, 2, 3, 3]), 0.0)

    def test_difference_region_counts_pixels_and_box(self):
        background = np.zeros((100, 100, 3), dtype=np.uint8)
        foreground = background.copy()
        foreground[45:55, 50:70] = 255
        box, count, _mask = difference_region(
            background, foreground, center=(60, 50), radius=30
        )
        self.assertEqual(box, [50, 45, 70, 55])
        self.assertEqual(count, 200)


if __name__ == "__main__":
    unittest.main()

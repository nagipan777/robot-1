import unittest

from evaluate_yolo_holdout import average_precision, iou


class YoloHoldoutEvaluationTests(unittest.TestCase):
    def test_iou(self):
        self.assertEqual(iou([0, 0, 10, 10], [0, 0, 10, 10]), 1.0)
        self.assertEqual(iou([0, 0, 2, 2], [3, 3, 4, 4]), 0.0)

    def test_average_precision_perfect_ranking(self):
        ranked = [{"tp": 1}, {"tp": 1}, {"tp": 0}]
        self.assertAlmostEqual(average_precision(ranked, 2), 1.0)


if __name__ == "__main__":
    unittest.main()

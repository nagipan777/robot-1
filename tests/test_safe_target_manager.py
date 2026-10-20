import unittest

from simulation.target_manager import (
    SafeTargetManager,
    evaluate_robot_guard,
    format_robot_proximity_log,
    require_fresh_timestamp,
    select_best_detection,
)


class SafeTargetManagerTests(unittest.TestCase):
    def setUp(self):
        self.manager = SafeTargetManager()

    def test_robot_detection_inside_guard_is_rejected(self):
        decision = evaluate_robot_guard((0.10, 0.0), (0.0, 0.0), 0.5)
        self.assertFalse(decision["accepted"])
        self.assertEqual(decision["reason"], "robot_proximity")
        log = format_robot_proximity_log(
            "map", (0.10, 0.0), 0.91, decision["distance"], 1234.5
        )
        for expected in (
            "reason=robot_proximity",
            "target_map=(0.100, 0.000)",
            "confidence=0.910",
            "distance=0.100m",
            "timestamp=1234.500000",
        ):
            self.assertIn(expected, log)

    def test_litter_outside_guard_is_accepted(self):
        decision = evaluate_robot_guard((0.80, 0.0), (0.0, 0.0), 0.5)
        self.assertTrue(decision["accepted"])
        self.assertIsNone(decision["reason"])

    def test_real_litter_inside_guard_is_safely_rejected(self):
        decision = evaluate_robot_guard((0.30, 0.40), (0.0, 0.0), 0.5)
        self.assertFalse(decision["accepted"])
        self.assertEqual(decision["reason"], "robot_proximity")
        self.assertAlmostEqual(decision["distance"], 0.5)

    def test_bbox_jitter_is_same_target(self):
        self.assertEqual(self.manager.consider((1.0, 2.0), 0.10), "new")
        self.assertEqual(self.manager.consider((1.04, 2.03), 0.10), "same")
        self.assertEqual(self.manager.reference_target, (1.0, 2.0))

    def test_reached_target_redetection_is_blocked(self):
        self.manager.consider((1.0, 2.0), 0.10)
        self.manager.mark_reached()
        self.assertEqual(
            self.manager.consider((1.02, 1.98), 0.10), "blocked_same"
        )
        self.assertTrue(self.manager.reached)

    def test_only_sufficiently_distant_target_releases_latch(self):
        self.manager.consider((1.0, 2.0), 0.10)
        self.manager.mark_reached()
        self.assertEqual(self.manager.consider((1.20, 2.0), 0.10), "new")
        self.assertFalse(self.manager.reached)

    def test_detection_loss_blocks_same_target_but_allows_new_target(self):
        self.manager.consider((1.0, 2.0), 0.10)
        self.manager.suspend()
        self.assertEqual(
            self.manager.consider((1.01, 2.01), 0.10), "blocked_same"
        )
        self.assertEqual(self.manager.consider((1.20, 2.0), 0.10), "new")
        self.assertFalse(self.manager.suspended)

    def test_invalid_change_distance_is_rejected(self):
        with self.assertRaises(ValueError):
            self.manager.consider((1.0, 2.0), -0.01)

    def test_confidence_gate_selects_only_qualified_litter(self):
        detections = [
            {"label": "litter", "confidence": 0.24},
            {"label": "other", "confidence": 0.99},
            {"label": "litter", "confidence": 0.81},
        ]
        selected = select_best_detection(detections, "litter", 0.25)
        self.assertEqual(selected["confidence"], 0.81)
        self.assertIsNone(
            select_best_detection(detections[:1], "litter", 0.25)
        )

    def test_stale_timestamp_is_rejected(self):
        self.assertEqual(require_fresh_timestamp(99.5, 100.0, 1.0), 99.5)
        with self.assertRaisesRegex(ValueError, "stale detection"):
            require_fresh_timestamp(98.0, 100.0, 1.0)


if __name__ == "__main__":
    unittest.main()

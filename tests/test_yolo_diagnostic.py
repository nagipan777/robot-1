import unittest
from unittest import mock

from diagnose_sim_litter import (
    THRESHOLDS,
    VARIANTS,
    wait_for_rendered_litter,
    world_to_pixel,
)


class YoloDiagnosticTests(unittest.TestCase):
    def test_required_confidence_thresholds_and_varied_poses(self):
        self.assertEqual(THRESHOLDS, (0.1, 0.25, 0.5))
        self.assertGreaterEqual(len(VARIANTS), 3)
        self.assertGreater(len({item["yaw"] for item in VARIANTS}), 1)
        self.assertGreater(len({(item["x"], item["y"]) for item in VARIANTS}), 1)

    def test_known_world_position_projects_near_previous_pixel(self):
        pixel_x, pixel_y = world_to_pixel(1.5, -0.7)
        self.assertAlmostEqual(pixel_x, 375.6, delta=1.0)
        self.assertAlmostEqual(pixel_y, 119.4, delta=1.0)

    @mock.patch("diagnose_sim_litter.time.sleep")
    @mock.patch("diagnose_sim_litter.difference_region")
    @mock.patch("diagnose_sim_litter.capture")
    @mock.patch("diagnose_sim_litter.wait_frames")
    def test_waits_for_entity_to_appear_in_rendered_frame(
        self, wait_frames, capture, difference_region, _sleep
    ):
        wait_frames.side_effect = [12, 14]
        capture.side_effect = ["stale", "rendered"]
        difference_region.side_effect = [
            ValueError("no difference"),
            ([1, 2, 3, 4], 8, "mask"),
        ]

        result = wait_for_rendered_litter(
            "http://camera/snapshot.jpg", 10, "background", (2, 3)
        )

        self.assertEqual(result, (14, "rendered", [1, 2, 3, 4], 8, "mask"))
        self.assertEqual(capture.call_count, 2)

    @mock.patch("diagnose_sim_litter.time.sleep")
    @mock.patch("diagnose_sim_litter.difference_region")
    @mock.patch("diagnose_sim_litter.capture")
    @mock.patch("diagnose_sim_litter.wait_frames")
    def test_rejects_a_stale_pose_away_from_expected_center(
        self, wait_frames, capture, difference_region, _sleep
    ):
        wait_frames.side_effect = [12, 14]
        capture.side_effect = ["old", "new"]
        difference_region.side_effect = [
            ([20, 20, 30, 30], 8, "old-mask"),
            ([0, 0, 10, 10], 8, "new-mask"),
        ]
        result = wait_for_rendered_litter(
            "http://camera/snapshot.jpg",
            10,
            "background",
            (5, 5),
            max_center_error=3,
        )
        self.assertEqual(result, (14, "new", [0, 0, 10, 10], 8, "new-mask"))


if __name__ == "__main__":
    unittest.main()

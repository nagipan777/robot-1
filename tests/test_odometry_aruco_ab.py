import math
import unittest

from simulation.localization_core import Pose2D
from simulation.odometry_aruco_ab import (
    TimedPose,
    compare_trajectories,
    interpolate,
    relative_trajectory,
)


class OdometryArucoComparisonTests(unittest.TestCase):
    def test_map_origin_and_heading_are_removed_before_comparison(self):
        odom = [TimedPose(1.0, Pose2D()), TimedPose(2.0, Pose2D(1.0, 0.0, 0.0))]
        aruco = [
            TimedPose(1.0, Pose2D(2.0, 3.0, math.pi / 2)),
            TimedPose(2.0, Pose2D(2.0, 4.0, math.pi / 2)),
        ]
        rows, metrics = compare_trajectories(odom, aruco)
        self.assertEqual(metrics["matched_samples"], 2)
        self.assertAlmostEqual(metrics["position_rmse_m"], 0.0, places=12)
        self.assertAlmostEqual(metrics["yaw_rmse_rad"], 0.0, places=12)
        self.assertAlmostEqual(rows[-1]["aruco_x_m"], 1.0)

    def test_linear_and_wrapped_yaw_interpolation(self):
        first = TimedPose(1.0, Pose2D(0.0, 0.0, math.radians(179)))
        second = TimedPose(3.0, Pose2D(2.0, 0.0, math.radians(-179)))
        midpoint = interpolate(first, second, 2.0)
        self.assertAlmostEqual(midpoint.x, 1.0)
        self.assertAlmostEqual(abs(midpoint.yaw), math.pi)

    def test_time_tolerance_counts_unmatched_samples(self):
        odom = [TimedPose(1.0, Pose2D()), TimedPose(2.0, Pose2D(1.0, 0.0, 0.0))]
        aruco = [TimedPose(1.0, Pose2D()), TimedPose(2.4, Pose2D(1.0, 0.0, 0.0))]
        rows, metrics = compare_trajectories(odom, aruco, max_time_delta_s=0.15)
        self.assertEqual(len(rows), 1)
        self.assertEqual(metrics["unmatched_odometry_samples"], 1)


if __name__ == "__main__":
    unittest.main()

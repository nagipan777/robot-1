import math
import unittest

from simulation.localization_core import (
    Pose2D,
    PoseFusion,
    RelativePoseComparator,
    WheelOdometry,
    decode_encoder_response,
    encoder_delta,
)
from simulation.target_manager import evaluate_robot_guard


class EncoderAndOdometryTests(unittest.TestCase):
    def estimator(self):
        return WheelOdometry(1.0, 1.0 / math.pi, 2.0, max_tick_delta=100)

    def test_packet_43_44_decode(self):
        self.assertEqual(decode_encoder_response(b"\x12\x34\xab\xcd"), (0x1234, 0xABCD))
        with self.assertRaises(ValueError):
            decode_encoder_response(b"\x00")

    def test_stop_and_straight(self):
        odom = self.estimator()
        self.assertFalse(odom.update(10, 10, 1.0).initialized)
        stopped = odom.update(10, 10, 2.0)
        self.assertEqual(stopped.pose, Pose2D())
        forward = odom.update(12, 12, 3.0)
        self.assertAlmostEqual(forward.pose.x, 2.0)
        self.assertAlmostEqual(forward.pose.yaw, 0.0)

    def test_left_right_and_in_place_turns(self):
        left = self.estimator()
        left.update(10, 10, 1.0)
        self.assertGreater(left.update(10, 12, 2.0).pose.yaw, 0.0)
        right = self.estimator()
        right.update(10, 10, 1.0)
        self.assertLess(right.update(12, 10, 2.0).pose.yaw, 0.0)
        turn = self.estimator()
        turn.update(10, 10, 1.0)
        result = turn.update(9, 11, 2.0)
        self.assertAlmostEqual(result.pose.x, 0.0)
        self.assertAlmostEqual(result.pose.y, 0.0)
        self.assertAlmostEqual(result.pose.yaw, 1.0)

    def test_wraparound(self):
        self.assertEqual(encoder_delta(2, 65534), 4)
        self.assertEqual(encoder_delta(65534, 2), -4)
        odom = self.estimator()
        odom.update(65534, 65534, 1.0)
        self.assertAlmostEqual(odom.update(2, 2, 2.0).pose.x, 4.0)

    def test_jump_bad_value_and_timestamp_are_rejected(self):
        odom = self.estimator()
        odom.update(10, 10, 1.0)
        with self.assertRaisesRegex(ValueError, "jump"):
            odom.update(200, 10, 2.0)
        with self.assertRaisesRegex(ValueError, "timestamp"):
            odom.update(11, 11, 1.0)
        with self.assertRaises((TypeError, ValueError)):
            odom.update(-1, 0, 3.0)

    def test_configured_velocity_limits_reject_without_committing_pose(self):
        odom = WheelOdometry(
            100.0,
            0.1,
            0.2,
            max_tick_delta=1000,
            max_linear_speed_m_s=0.1,
            max_angular_speed_rad_s=1.0,
        )
        odom.update(0, 0, 1.0)
        pose_before = odom.pose
        with self.assertRaisesRegex(ValueError, "linear velocity"):
            odom.update(100, 100, 1.1)
        self.assertEqual(odom.pose, pose_before)
        self.assertEqual(odom.previous, (0, 0))


class FusionTests(unittest.TestCase):
    def fusion(self, **overrides):
        parameters = dict(
            correction_gain=0.5,
            max_position_innovation_m=0.5,
            max_yaw_innovation_rad=0.8,
            max_aruco_age=0.5,
            max_odom_age=0.5,
            max_dead_reckon_age=2.0,
        )
        parameters.update(overrides)
        return PoseFusion(**parameters)

    def test_aruco_initializes_map_from_odom_transform(self):
        fusion = self.fusion()
        fusion.observe_odom(Pose2D(), 10.0)
        self.assertEqual(
            fusion.observe_aruco(Pose2D(1.0, 2.0, 0.3), 10.0, 10.0),
            (True, "initialized"),
        )
        pose, state, confidence = fusion.estimate(10.1)
        self.assertEqual(state, "FUSED")
        self.assertEqual(confidence, 1.0)
        self.assertAlmostEqual(pose.x, 1.0)
        self.assertAlmostEqual(pose.y, 2.0)

    def test_odometry_prediction_and_aruco_correction(self):
        fusion = self.fusion()
        fusion.observe_odom(Pose2D(), 10.0)
        fusion.observe_aruco(Pose2D(1.0, 0.0, 0.0), 10.0, 10.0)
        fusion.observe_odom(Pose2D(0.2, 0.0, 0.0), 10.2)
        self.assertAlmostEqual(fusion.estimate(10.2)[0].x, 1.2)
        accepted, reason = fusion.observe_aruco(Pose2D(1.3, 0.0, 0.0), 10.2, 10.2)
        self.assertTrue(accepted)
        self.assertEqual(reason, "corrected")
        self.assertAlmostEqual(fusion.estimate(10.2)[0].x, 1.25)

    def test_outliers_frames_and_stale_observations_are_rejected(self):
        fusion = self.fusion()
        fusion.observe_odom(Pose2D(), 10.0)
        fusion.observe_aruco(Pose2D(), 10.0, 10.0)
        self.assertEqual(
            fusion.observe_aruco(Pose2D(1.0, 0.0, 0.0), 10.1, 10.1)[1],
            "position_outlier",
        )
        self.assertEqual(
            fusion.observe_aruco(Pose2D(), 10.2, 10.2, "odom")[1],
            "frame_mismatch",
        )
        self.assertEqual(
            fusion.observe_aruco(Pose2D(), 9.0, 10.2)[1], "stale_aruco"
        )
        with self.assertRaisesRegex(ValueError, "frame"):
            fusion.observe_odom(Pose2D(), 10.2, "map")

    def test_sensor_loss_is_bounded(self):
        fusion = self.fusion(max_odom_age=5.0)
        fusion.observe_odom(Pose2D(), 10.0)
        fusion.observe_aruco(Pose2D(), 10.0, 10.0)
        fusion.observe_odom(Pose2D(0.1, 0.0, 0.0), 10.8)
        self.assertEqual(fusion.estimate(10.8)[1], "ODOM_ONLY")
        self.assertEqual(fusion.estimate(12.1)[1], "LOST")

    def test_stale_odom_blocks_pose_and_guard_uses_fused_map_position(self):
        fusion = self.fusion()
        fusion.observe_odom(Pose2D(), 10.0)
        fusion.observe_aruco(Pose2D(1.0, 1.0, 0.0), 10.0, 10.0)
        pose, state, _ = fusion.estimate(10.6)
        self.assertIsNone(pose)
        self.assertEqual(state, "STALE_ODOM")
        fusion.observe_odom(Pose2D(), 10.7)
        pose = fusion.estimate(10.7)[0]
        decision = evaluate_robot_guard((1.3, 1.0), (pose.x, pose.y), 0.5)
        self.assertFalse(decision["accepted"])
        self.assertEqual(decision["reason"], "robot_proximity")


class RelativePoseComparatorTests(unittest.TestCase):
    def test_aruco_is_compared_as_relative_motion_not_map_coordinates(self):
        comparator = RelativePoseComparator()
        comparator.observe_aruco(Pose2D(2.0, 3.0, math.pi / 2), 10.0)
        comparator.observe_aruco(Pose2D(2.0, 4.0, math.pi / 2), 11.0)
        result = comparator.compare(Pose2D(1.0, 0.0, 0.0), 11.1)
        self.assertAlmostEqual(result.aruco_relative.x, 1.0)
        self.assertAlmostEqual(result.aruco_relative.y, 0.0)
        self.assertAlmostEqual(result.position_error_m, 0.0)
        self.assertAlmostEqual(result.yaw_error_rad, 0.0)
        self.assertAlmostEqual(result.aruco_age_s, 0.1)

    def test_missing_and_non_monotonic_aruco_are_handled(self):
        comparator = RelativePoseComparator()
        self.assertIsNone(comparator.compare(Pose2D(), 1.0))
        comparator.observe_aruco(Pose2D(), 2.0)
        with self.assertRaisesRegex(ValueError, "increase"):
            comparator.observe_aruco(Pose2D(), 2.0)


if __name__ == "__main__":
    unittest.main()

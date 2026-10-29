import math
import tempfile
import unittest
from pathlib import Path

from simulation.localization_core import WheelOdometry
from simulation.odometry_dryrun_core import (
    DryRunOdometry,
    EncoderSample,
    load_encoder_csv,
    parse_encoder_sample,
)


class OdometryDryRunCoreTests(unittest.TestCase):
    def processor(self):
        estimator = WheelOdometry(1.0, 1.0 / math.pi, 2.0, max_tick_delta=100)
        return DryRunOdometry(estimator, max_time_gap_s=1.0)

    def sample(self, timestamp, left, right, sequence, **overrides):
        values = dict(valid=True, communication_state="connected", error="")
        values.update(overrides)
        return EncoderSample(timestamp, left, right, values["valid"], values["communication_state"], sequence, values["error"])

    def test_forward_reverse_turn_stop_and_deltas(self):
        processor = self.processor()
        self.assertEqual(processor.process(self.sample(1.0, 100, 100, 1)).reason, "baseline")
        forward = processor.process(self.sample(2.0, 110, 110, 2))
        self.assertTrue(forward.integrated)
        self.assertEqual((forward.update.left_delta, forward.update.right_delta), (10, 10))
        self.assertAlmostEqual(forward.update.pose.x, 10.0)
        stopped = processor.process(self.sample(2.5, 110, 110, 3))
        self.assertEqual((stopped.update.left_delta, stopped.update.right_delta), (0, 0))
        reverse = processor.process(self.sample(3.0, 105, 105, 4))
        self.assertLess(reverse.update.linear_velocity, 0)
        left_turn = processor.process(self.sample(4.0, 104, 106, 5))
        self.assertGreater(left_turn.update.angular_velocity, 0)
        right_turn = processor.process(self.sample(5.0, 106, 104, 6))
        self.assertLess(right_turn.update.angular_velocity, 0)

    def test_disconnect_timeout_and_large_gap_rebaseline_without_jump(self):
        processor = self.processor()
        processor.process(self.sample(1.0, 10, 10, 1))
        processor.process(self.sample(1.2, 11, 11, 2))
        lost = processor.process(self.sample(1.3, None, None, 3, valid=False, communication_state="timeout", error="rx_timeout"))
        self.assertFalse(lost.integrated)
        pose_before = processor.estimator.pose
        recovered = processor.process(self.sample(1.4, 500, 500, 4))
        self.assertEqual(recovered.reason, "baseline")
        self.assertEqual(recovered.update.pose, pose_before)
        gap = processor.process(self.sample(3.0, 700, 700, 5))
        self.assertEqual(gap.reason, "baseline")
        self.assertEqual(gap.update.pose, pose_before)

    def test_wraparound_and_invalid_jump(self):
        processor = self.processor()
        processor.process(self.sample(1.0, 65534, 65534, 1))
        wrapped = processor.process(self.sample(2.0, 2, 2, 2))
        self.assertEqual((wrapped.update.left_delta, wrapped.update.right_delta), (4, 4))
        with self.assertRaisesRegex(ValueError, "jump"):
            processor.process(self.sample(2.5, 500, 2, 3))

    def test_timestamp_regression_is_rejected_without_integration(self):
        processor = self.processor()
        processor.process(self.sample(2.0, 100, 100, 1))
        pose_before = processor.estimator.pose
        with self.assertRaisesRegex(ValueError, "timestamp"):
            processor.process(self.sample(1.9, 101, 101, 2))
        self.assertEqual(processor.estimator.pose, pose_before)

    def test_csv_validation_rejects_duplicate_time_and_sequence(self):
        content = "timestamp,left_encoder,right_encoder,valid,communication_state,sequence,error\n1,10,10,true,connected,1,\n1,11,11,true,connected,2,\n"
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "input.csv"
            path.write_text(content)
            with self.assertRaisesRegex(ValueError, "timestamp"):
                load_encoder_csv(path)
        with self.assertRaisesRegex(ValueError, "requires both"):
            parse_encoder_sample({"timestamp": "1", "left_encoder": "", "right_encoder": "1", "valid": "true", "communication_state": "connected", "sequence": "1"})

    def test_repository_scenarios_have_expected_motion(self):
        root = Path(__file__).resolve().parents[1] / "docs" / "odometry_dryrun_inputs"
        expectations = {
            "forward.csv": lambda pose: pose.x > 0 and abs(pose.yaw) < 1e-12,
            "reverse.csv": lambda pose: pose.x < 0 and abs(pose.yaw) < 1e-12,
            "left_turn.csv": lambda pose: pose.yaw > 0,
            "right_turn.csv": lambda pose: pose.yaw < 0,
            "stopped.csv": lambda pose: pose.x == pose.y == pose.yaw == 0,
            "wraparound.csv": lambda pose: pose.x > 0,
            "disconnect_reconnect.csv": lambda pose: 0 < pose.x < 0.02,
        }
        for name, predicate in expectations.items():
            with self.subTest(name=name):
                estimator = WheelOdometry(508.8, 0.072, 0.235, max_tick_delta=2000)
                processor = DryRunOdometry(estimator)
                for sample in load_encoder_csv(root / name):
                    processor.process(sample)
                self.assertTrue(predicate(estimator.pose), estimator.pose)

    def test_theoretical_turn_is_labelled_and_about_26_8_degrees(self):
        path = Path(__file__).resolve().parents[1] / "docs" / "odometry_dryrun_inputs" / "turn_26_8_theoretical.csv"
        estimator = WheelOdometry(508.8, 0.072, 0.235, max_tick_delta=2000)
        processor = DryRunOdometry(estimator)
        for sample in load_encoder_csv(path):
            processor.process(sample)
        self.assertAlmostEqual(math.degrees(estimator.pose.yaw), 26.8, delta=0.2)

    def test_repository_abnormal_jump_is_rejected(self):
        path = Path(__file__).resolve().parents[1] / "docs" / "odometry_dryrun_inputs" / "abnormal_jump.csv"
        processor = DryRunOdometry(
            WheelOdometry(508.8, 0.072, 0.235, max_tick_delta=2000)
        )
        samples = load_encoder_csv(path)
        processor.process(samples[0])
        processor.process(samples[1])
        with self.assertRaisesRegex(ValueError, "jump"):
            processor.process(samples[2])


if __name__ == "__main__":
    unittest.main()

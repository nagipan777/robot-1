import unittest

from simulation.odometry_dryrun_core import EncoderSample
from simulation.wheel_motion_monitor import WheelMotionMonitor


def sample(at, left, right, sequence=1, *, valid=True, state="connected", error=""):
    return EncoderSample(at, left, right, valid, state, sequence, error)


class WheelMotionMonitorTests(unittest.TestCase):
    def monitor(self, **overrides):
        values = dict(
            encoder_counts_per_rev=100.0,
            wheel_diameter_m=1.0 / 3.141592653589793,
            wheel_separation_m=2.0,
            command_no_motion_grace_s=0.2,
            stop_motion_grace_s=0.15,
            measurement_stale_s=0.5,
            max_tick_delta=100,
            started_at_s=0.0,
        )
        values.update(overrides)
        return WheelMotionMonitor(**values)

    def test_straight_and_turn_estimates(self):
        monitor = self.monitor()
        monitor.observe_encoder(sample(1.0, 100, 100), 1.0)
        straight = monitor.observe_encoder(sample(1.1, 110, 110, 2), 1.1)
        self.assertEqual(straight.event, "encoder_update")
        self.assertEqual((straight.left_delta, straight.right_delta), (10, 10))
        self.assertAlmostEqual(straight.estimated_distance_m, 0.1)
        turn = monitor.observe_encoder(sample(1.2, 105, 115, 3), 1.2)
        self.assertAlmostEqual(turn.estimated_distance_m, 0.0)
        self.assertGreater(turn.estimated_yaw_rad, 0.0)

    def test_command_without_encoder_change_is_warning_after_grace(self):
        monitor = self.monitor()
        monitor.observe_encoder(sample(1.0, 100, 100), 1.0)
        monitor.observe_command(20, 20, 1.0)
        event = monitor.observe_encoder(sample(1.3, 100, 100, 2), 1.3)
        self.assertEqual(event.event, "command_without_encoder_change")
        self.assertEqual(event.severity, "warning")

    def test_stop_coast_and_late_change_are_distinct(self):
        monitor = self.monitor()
        monitor.observe_encoder(sample(1.0, 100, 100), 1.0)
        monitor.observe_command(20, 20, 1.0)
        monitor.observe_encoder(sample(1.1, 102, 102, 2), 1.1)
        monitor.observe_command(0, 0, 1.2)
        coast = monitor.observe_encoder(sample(1.3, 103, 103, 3), 1.3)
        self.assertEqual(coast.event, "stop_coast_observed")
        late = monitor.observe_encoder(sample(1.5, 104, 104, 4), 1.5)
        self.assertEqual(late.event, "wheel_change_after_stop")
        self.assertEqual(late.severity, "warning")

    def test_stale_or_missing_is_not_zero_motion(self):
        monitor = self.monitor()
        missing = monitor.poll(0.6)
        self.assertEqual(missing.event, "measurement_stale")
        self.assertIsNone(missing.left_delta)
        stale = monitor.observe_encoder(sample(1.0, 100, 100), 1.7)
        self.assertEqual(stale.event, "measurement_stale")
        self.assertIsNone(stale.left_delta)

    def test_disconnect_and_reconnect_rebaseline_without_motion(self):
        monitor = self.monitor()
        monitor.observe_encoder(sample(1.0, 100, 100), 1.0)
        lost = monitor.observe_encoder(
            sample(1.1, None, None, 2, valid=False, state="timeout", error="rx_timeout"),
            1.1,
        )
        self.assertEqual(lost.event, "measurement_unavailable")
        recovered = monitor.observe_encoder(sample(1.2, 50000, 50000, 3), 1.2)
        self.assertEqual(recovered.event, "encoder_baseline")
        self.assertIsNone(recovered.estimated_distance_m)

    def test_wraparound_and_jump_rebaseline(self):
        monitor = self.monitor()
        monitor.observe_encoder(sample(1.0, 65534, 65534), 1.0)
        wrapped = monitor.observe_encoder(sample(1.1, 2, 2, 2), 1.1)
        self.assertEqual((wrapped.left_delta, wrapped.right_delta), (4, 4))
        jump = monitor.observe_encoder(sample(1.2, 500, 2, 3), 1.2)
        self.assertEqual(jump.event, "encoder_jump_rebaseline")
        self.assertIsNone(jump.estimated_distance_m)

    def test_arm_state_is_request_not_ack(self):
        event = self.monitor().observe_arm_request(123, 1.0)
        self.assertEqual(event.motion_state, "arm_requested")
        self.assertIn("ACK not observed", event.detail)


if __name__ == "__main__":
    unittest.main()

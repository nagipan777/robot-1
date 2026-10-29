import csv
import json
import tempfile
import unittest
from pathlib import Path

from simulation.encoder_stop_interval_core import (
    acquire_stopped_encoder,
    evaluate_interval,
    require_after_gate,
    require_drive_approval,
)
from simulation import run_encoder_stop_interval


class FakeTransport:
    def __init__(self, *, stop=(7, 0, 0), mode=2, counts=(100, 200), error=None):
        self.stop = stop
        self.mode = mode
        self.counts = counts
        self.error = error
        self.closed = False
        self.arm_calls = 0
        self.drive_calls = 0

    def connect(self): pass
    def verify_stop(self): return self.stop
    def query_oi_mode(self): return self.mode
    def query_encoders(self):
        if self.error:
            raise self.error
        return self.counts
    def close(self): self.closed = True


class EncoderStopIntervalTests(unittest.TestCase):
    def test_harness_has_no_arm_drive_or_network_owner(self):
        root = Path(__file__).resolve().parents[1] / "simulation"
        source = (root / "run_encoder_stop_interval.py").read_text()
        core = (root / "encoder_stop_interval_core.py").read_text()
        combined = source + core
        self.assertNotIn(".arm(", combined)
        self.assertNotIn(".send_twist(", combined)
        self.assertNotIn("create_connection", combined)

    def test_same_direction_and_measured_error(self):
        result = evaluate_interval(100, 200, 110, 210, measured_distance_m=0.005)
        self.assertEqual((result.left_delta_ticks, result.right_delta_ticks), (10, 10))
        self.assertAlmostEqual(result.estimated_yaw_rad, 0.0)
        self.assertAlmostEqual(result.distance_error_m, result.estimated_distance_m - 0.005)

    def test_opposite_direction_ticks_produce_rotation(self):
        result = evaluate_interval(100, 200, 90, 210, expected_left_sign=-1, expected_right_sign=1)
        self.assertAlmostEqual(result.estimated_distance_m, 0.0)
        self.assertGreater(result.estimated_yaw_rad, 0.0)
        self.assertTrue(result.left_sign_matches)
        self.assertTrue(result.right_sign_matches)

    def test_wraparound_and_abnormal_jump(self):
        wrapped = evaluate_interval(65534, 2, 2, 65534)
        self.assertEqual((wrapped.left_delta_ticks, wrapped.right_delta_ticks), (4, -4))
        abnormal = evaluate_interval(0, 0, 3000, 0, max_tick_delta=2000)
        self.assertTrue(abnormal.anomalous_jump)

    def test_stop_failure_and_encoder_failure_abort_without_retry(self):
        stopped = FakeTransport(stop=(3, 0, 0))
        with self.assertRaisesRegex(RuntimeError, "STOP/Safe"):
            acquire_stopped_encoder(stopped)
        self.assertTrue(stopped.closed)
        failed = FakeTransport(error=ConnectionError("partial response"))
        with self.assertRaisesRegex(ConnectionError, "partial"):
            acquire_stopped_encoder(failed)
        self.assertTrue(failed.closed)

    def test_drive_not_approved_and_physical_stop_not_confirmed_are_rejected(self):
        session = {"state": "before_acquired", "drive_approved": False}
        with self.assertRaises(PermissionError):
            require_after_gate(session, "PHYSICAL_STOP_CONFIRMED")
        with self.assertRaises(PermissionError):
            require_drive_approval(session, "wrong")
        require_drive_approval(session, "ATTENDED_DRIVE_APPROVED")
        with self.assertRaises(PermissionError):
            require_after_gate(session, "wrong")
        require_after_gate(session, "PHYSICAL_STOP_CONFIRMED")

    def test_offline_cli_round_trip_and_invalid_csv_rejection(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            before = root / "before.csv"
            after = root / "after.csv"
            fields = ["timestamp", "left_encoder", "right_encoder", "valid", "communication_state", "sequence", "error"]
            for path, row in ((before, [1, 100, 200, True, "connected", 1, ""]), (after, [2, 110, 210, True, "connected", 1, ""])):
                with path.open("w", newline="") as target:
                    writer = csv.writer(target)
                    writer.writerow(fields)
                    writer.writerow(row)
            session = root / "session.json"
            run_encoder_stop_interval.prepare(type("Args", (), {"before_csv": str(before), "session": str(session), "motion_label": "straight"})())
            state = json.loads(session.read_text())
            require_drive_approval(state, "ATTENDED_DRIVE_APPROVED")
            session.write_text(json.dumps(state))
            args = type("Args", (), {
                "session": str(session), "after_csv": str(after), "output_csv": str(root / "result.csv"),
                "summary": str(root / "summary.json"), "confirmation": "PHYSICAL_STOP_CONFIRMED",
                "encoder_counts_per_rev": 508.8, "wheel_diameter_m": 0.072, "wheel_separation_m": 0.235,
                "max_tick_delta": 2000, "measured_distance_m": None, "measured_yaw_rad": None,
                "expected_left_sign": 1, "expected_right_sign": 1,
            })()
            run_encoder_stop_interval.finalize(args)
            self.assertTrue((root / "result.csv").exists())
            bad = root / "bad.csv"
            bad.write_text(",".join(fields) + "\n3,,,False,timeout,1,disconnect\n")
            with self.assertRaisesRegex(RuntimeError, "invalid"):
                run_encoder_stop_interval.stable_endpoint(bad, "after")


if __name__ == "__main__":
    unittest.main()

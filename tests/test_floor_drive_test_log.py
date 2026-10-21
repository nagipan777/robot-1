import csv
import json
import tempfile
import unittest
from pathlib import Path

from floor_drive_test_log import FloorDriveRecord


class FloorDriveTestLogTests(unittest.TestCase):
    def test_writes_matching_json_and_csv_without_hardware(self):
        record = FloorDriveRecord("floor-20261020-001", "stage1")
        record.add_event(
            "drive_ack",
            arm_generation=100,
            drive_sequence=1,
            left_mm_s=20,
            right_mm_s=20,
            uart_ack="0x03",
            watchdog_state="ARMED",
        )
        record.add_event(
            "stop",
            stop_reason="watchdog",
            physical_stop_observed=True,
            encoder_delta_left=2,
            encoder_delta_right=2,
        )
        record.finalize("PASS")
        with tempfile.TemporaryDirectory() as directory:
            json_path, csv_path = record.write(directory)
            payload = json.loads(json_path.read_text(encoding="utf-8"))
            with csv_path.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
        self.assertEqual(payload["test_id"], "floor-20261020-001")
        self.assertEqual(payload["result"], "PASS")
        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[0]["drive_sequence"], "1")
        self.assertEqual(rows[1]["stop_reason"], "watchdog")

    def test_rejects_invalid_stage_result_and_test_id(self):
        with self.assertRaises(ValueError):
            FloorDriveRecord("bad id", "stage0")
        with self.assertRaises(ValueError):
            FloorDriveRecord("id", "stage9")
        record = FloorDriveRecord("id", "stage0")
        with self.assertRaises(ValueError):
            record.finalize("SUCCESS")

    def test_default_record_is_locked_and_not_run(self):
        record = FloorDriveRecord("id", "stage0")
        self.assertEqual(record.result, "NOT_RUN")
        self.assertTrue(all(value is False for value in record.safety_flags.values()))
        self.assertEqual(record.events, [])


if __name__ == "__main__":
    unittest.main()

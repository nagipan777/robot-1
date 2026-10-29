import csv
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from simulation import roomba_encoder_probe


class FakeTransport:
    samples = [(100, 200), (101, 201)]
    instances = []

    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.connected = False
        self.stop_verified = False
        self.closed = False
        self.sample_index = 0
        self.__class__.instances.append(self)

    def connect(self):
        self.connected = True

    def verify_stop(self):
        self.stop_verified = True
        return 7, 0, 0

    def query_oi_mode(self):
        return 2

    def query_encoders(self):
        sample = self.samples[self.sample_index]
        self.sample_index += 1
        return sample

    def close(self):
        self.closed = True


class RoombaEncoderProbeTests(unittest.TestCase):
    def setUp(self):
        FakeTransport.instances.clear()
        FakeTransport.samples = [(100, 200), (101, 201)]

    def test_static_probe_requires_stop_and_safe_then_writes_csv(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "encoders.csv"
            with patch.object(
                roomba_encoder_probe, "RoombaTcpTransport", FakeTransport
            ):
                rows = roomba_encoder_probe.run(
                    "pico", 9000, 2, 0.0, output
                )

            transport = FakeTransport.instances[0]
            self.assertTrue(transport.connected)
            self.assertTrue(transport.stop_verified)
            self.assertTrue(transport.closed)
            self.assertEqual(
                [(row["left_encoder"], row["right_encoder"]) for row in rows],
                [(100, 200), (101, 201)],
            )
            with output.open(encoding="utf-8") as source:
                saved = list(csv.DictReader(source))
            self.assertEqual(len(saved), 2)
            self.assertEqual(saved[0]["valid"], "true")

    def test_sensor_failure_is_recorded_invalid_and_connection_is_closed(self):
        class FailingTransport(FakeTransport):
            def query_encoders(self):
                raise TimeoutError("encoder timeout")

        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "encoders.csv"
            with patch.object(
                roomba_encoder_probe, "RoombaTcpTransport", FailingTransport
            ):
                with self.assertRaisesRegex(TimeoutError, "encoder timeout"):
                    roomba_encoder_probe.run("pico", 9000, 1, 0.0, output)

            with output.open(encoding="utf-8") as source:
                saved = list(csv.DictReader(source))
            self.assertEqual(saved[0]["valid"], "false")
            self.assertEqual(saved[0]["communication_state"], "error")
            self.assertIn("encoder timeout", saved[0]["error"])
            self.assertTrue(FailingTransport.instances[-1].closed)

    def test_probe_source_contains_no_arm_or_drive_call(self):
        source = Path(roomba_encoder_probe.__file__).read_text(encoding="utf-8")
        self.assertNotIn(".arm(", source)
        self.assertNotIn(".send_twist(", source)


if __name__ == "__main__":
    unittest.main()

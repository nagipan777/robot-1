import math
import time
import unittest
from unittest.mock import patch

from roomba_driver import (
    RoombaCommunicationError,
    RoombaConnectionError,
    RoombaDriver,
    RoombaSafetyError,
    _run_odometry_test,
    main,
)


STOP = bytes([145, 0, 0, 0, 0])


class FakeSerial:
    def __init__(self, reads=(), fail_on_write=None):
        self.reads = list(reads)
        self.writes = []
        self.fail_on_write = fail_on_write
        self.write_count = 0
        self.is_open = True
        self.closed = False

    def write(self, data):
        self.write_count += 1
        if self.fail_on_write == self.write_count:
            raise OSError("serial write failed")
        self.writes.append(bytes(data))
        return len(data)

    def flush(self):
        pass

    def read(self, size):
        if not self.reads:
            return b""
        chunk = self.reads.pop(0)
        if len(chunk) > size:
            self.reads.insert(0, chunk[size:])
            return chunk[:size]
        return chunk

    def reset_input_buffer(self):
        pass

    def close(self):
        self.closed = True
        self.is_open = False


def connected_driver(fake, **kwargs):
    driver = RoombaDriver(
        serial_instance=fake,
        read_timeout=kwargs.pop("read_timeout", 0.02),
        command_timeout=kwargs.pop("command_timeout", None),
        **kwargs,
    )
    driver._connected = True
    driver._safe_confirmed = True
    return driver


class RoombaDriverTests(unittest.TestCase):
    def test_connect_confirms_safe_mode(self):
        fake = FakeSerial([bytes([2])])
        driver = RoombaDriver(
            serial_instance=fake, read_timeout=0.02, command_timeout=None
        )
        with patch("roomba_driver.time.sleep"):
            driver.connect()
        self.assertTrue(driver.safe_confirmed)
        self.assertEqual(
            fake.writes[:5],
            [
                bytes([128]),
                bytes([128]),
                bytes([128]),
                bytes([131]),
                bytes([142, 35]),
            ],
        )
        driver.close()

    def test_unconfirmed_safe_mode_prevents_drive(self):
        fake = FakeSerial([bytes([1])])
        driver = RoombaDriver(
            serial_instance=fake, read_timeout=0.02, command_timeout=None
        )
        with patch("roomba_driver.time.sleep"):
            with self.assertRaises(RoombaConnectionError):
                driver.connect()
        writes_after_connect = len(fake.writes)
        with self.assertRaises(RoombaSafetyError):
            driver.drive_direct(10, 10)
        self.assertEqual(len(fake.writes), writes_after_connect)

    def test_drive_direct_bytes_are_right_then_left_and_signed(self):
        fake = FakeSerial()
        driver = connected_driver(fake)
        driver.drive_direct(left_mm_s=-50, right_mm_s=25)
        self.assertEqual(fake.writes[-1], bytes([145, 0, 25, 255, 206]))

    def test_speed_limit_rejects_command_without_writing(self):
        fake = FakeSerial()
        driver = connected_driver(fake, max_speed_mm_s=50)
        with self.assertRaises(RoombaSafetyError):
            driver.drive_direct(51, 0)
        self.assertEqual(fake.writes, [])

    def test_encoder_decode_collects_partial_reads(self):
        fake = FakeSerial([b"\x12", b"\x34\xab", b"\xcd"])
        driver = connected_driver(fake)
        self.assertEqual(driver.read_encoders(), (0x1234, 0xABCD))
        self.assertEqual(fake.writes[0], bytes([149, 2, 43, 44]))

    def test_read_odometry_updates_from_encoder_readings(self):
        fake = FakeSerial([b"\x00\x64\x00\xc8", b"\x00\x6e\x00\xd2"])
        driver = connected_driver(fake)
        initial = driver.read_odometry()
        self.assertEqual((initial.x, initial.y, initial.theta), (0.0, 0.0, 0.0))

        pose = driver.read_odometry()
        expected_distance = 10 * math.pi * 0.072 / 508.8
        self.assertAlmostEqual(pose.x, expected_distance)
        self.assertAlmostEqual(pose.y, 0.0)
        self.assertAlmostEqual(pose.theta, 0.0)

    def test_odom_cli_reports_ten_samples_without_motion_command(self):
        encoder_pairs = [(65534, 100)] + [
            ((65534 + 4 * index) % 65536, 100 + 4 * index)
            for index in range(1, 11)
        ]
        reads = [left.to_bytes(2, "big") + right.to_bytes(2, "big")
                 for left, right in encoder_pairs]
        fake = FakeSerial(reads)
        driver = connected_driver(fake)
        lines = []

        with patch("roomba_driver.time.sleep") as sleep:
            result = _run_odometry_test(driver, output=lines.append)

        self.assertEqual(result, 0)
        self.assertEqual(sleep.call_count, 10)
        sleep.assert_called_with(0.5)
        self.assertEqual(len(lines), 11)
        self.assertEqual(lines[0], "Baseline: L=65534 R=100")
        self.assertIn("L=2 R=104 dL=+4 dR=+4", lines[1])
        self.assertIn("x=", lines[-1])
        self.assertIn("y=", lines[-1])
        self.assertIn("theta=", lines[-1])
        self.assertFalse(
            any(write[0] == 145 for write in fake.writes),
            "odometry test must not send Drive Direct",
        )

    def test_odom_cli_aborts_without_pose_update_on_partial_receive(self):
        fake = FakeSerial([b"\x00\x64\x00\xc8", b"\x00\x65"])
        driver = connected_driver(fake, read_timeout=0.005)
        lines = []

        with patch("roomba_driver.time.sleep"):
            result = _run_odometry_test(driver, output=lines.append)

        self.assertEqual(result, 1)
        self.assertEqual((driver.odometry.prev_left, driver.odometry.prev_right), (100, 200))
        self.assertIn("incomplete response", lines[-1])
        self.assertFalse(driver.safe_confirmed)

    def test_odom_cli_option_routes_to_read_only_test(self):
        with patch("roomba_driver.RoombaDriver") as driver_class:
            connected = driver_class.return_value.__enter__.return_value
            with patch("roomba_driver._run_odometry_test", return_value=0) as run:
                result = main(["--port", "test-port", "--odom-test"])

        self.assertEqual(result, 0)
        driver_class.assert_called_once_with(port="test-port")
        run.assert_called_once_with(connected)

    def test_encoder_timeout_is_not_returned_as_success_and_stops(self):
        fake = FakeSerial([b"\x12\x34"])
        driver = connected_driver(fake, read_timeout=0.005)
        with self.assertRaises(RoombaCommunicationError):
            driver.read_encoders()
        self.assertIn(STOP, fake.writes)
        self.assertFalse(driver.safe_confirmed)

    def test_write_error_attempts_stop(self):
        fake = FakeSerial(fail_on_write=1)
        driver = connected_driver(fake)
        with self.assertRaises(RoombaCommunicationError):
            driver.drive_direct(10, 10)
        self.assertIn(STOP, fake.writes)

    def test_close_sends_stop_and_closes_port(self):
        fake = FakeSerial()
        driver = connected_driver(fake)
        driver.close()
        self.assertEqual(fake.writes[-1], STOP)
        self.assertTrue(fake.closed)

    def test_watchdog_stops_after_command_timeout(self):
        fake = FakeSerial([bytes([2])])
        driver = RoombaDriver(
            serial_instance=fake,
            read_timeout=0.02,
            command_timeout=0.02,
        )
        with patch("roomba_driver.time.sleep"):
            driver.connect()
        driver.drive_direct(10, 10)
        time.sleep(0.08)
        self.assertIn(STOP, fake.writes[4:])
        driver.close()


if __name__ == "__main__":
    unittest.main()

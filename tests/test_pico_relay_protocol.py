import importlib.util
import io
import errno
import pathlib
import sys
import types
import unittest
from contextlib import redirect_stdout
from unittest import mock


SOURCE = pathlib.Path(__file__).resolve().parents[1] / "pico" / "pico_relay.py"


class FakeUart:
    def __init__(self):
        self.writes = []
        self.read_buffer = bytearray()
        self.responses = {}
        self.partial_write_for = None
        self.write_result_for = {}

    def write(self, data):
        data = bytes(data)
        self.writes.append(data)
        self.read_buffer.extend(self.responses.get(data, b""))
        if data in self.write_result_for:
            return self.write_result_for[data]
        if data == self.partial_write_for:
            return max(0, len(data) - 1)
        return len(data)

    def any(self):
        return len(self.read_buffer)

    def read(self, size):
        result = bytes(self.read_buffer[:size])
        self.read_buffer = self.read_buffer[size:]
        return result

    def deinit(self):
        pass


class FakeConnection:
    def __init__(self, max_send=None):
        self.sent = bytearray()
        self.max_send = max_send

    def send(self, data):
        size = len(data) if self.max_send is None else min(self.max_send, len(data))
        self.sent.extend(data[:size])
        return size


class TemporarilyBlockedConnection(FakeConnection):
    def __init__(self, blocked_sends):
        super().__init__()
        self.blocked_sends = blocked_sends

    def send(self, data):
        if self.blocked_sends:
            self.blocked_sends -= 1
            raise OSError(errno.EAGAIN)
        return super().send(data)


class NoneThenWritableConnection(FakeConnection):
    def __init__(self, none_sends):
        super().__init__()
        self.none_sends = none_sends

    def send(self, data):
        if self.none_sends:
            self.none_sends -= 1
            return None
        return super().send(data)


class FakeTime:
    def __init__(self):
        self.now = 0

    def ticks_ms(self):
        self.now += 2
        return self.now

    @staticmethod
    def ticks_diff(current, started):
        return current - started

    @staticmethod
    def sleep_ms(_milliseconds):
        pass


def load_relay():
    uart = FakeUart()

    class FakePin:
        OUT = 1

        def __init__(self, *_args, **_kwargs):
            self.transitions = []

        def on(self):
            self.transitions.append(1)

        def off(self):
            self.transitions.append(0)

    machine = types.ModuleType("machine")
    machine.Pin = FakePin
    machine.UART = lambda *_args, **_kwargs: uart

    class FakeWlan:
        def active(self, *_args):
            pass

        def isconnected(self):
            return True

        def ifconfig(self):
            return ("127.0.0.1", "", "", "")

    network = types.ModuleType("network")
    network.STA_IF = 0
    network.WLAN = lambda *_args: FakeWlan()

    spec = importlib.util.spec_from_file_location("pico_relay_protocol_test", SOURCE)
    module = importlib.util.module_from_spec(spec)
    with mock.patch.dict(sys.modules, {"machine": machine, "network": network}):
        spec.loader.exec_module(module)
    module.time = FakeTime()
    return module, uart


class PicoRelayProtocolTests(unittest.TestCase):
    def test_uart_write_exact_requires_the_complete_return_count(self):
        frame = b"\x91\x00\x14\x00\x14"
        for result, expected in ((5, True), (4, False), (0, False), (None, False)):
            with self.subTest(result=result):
                relay, uart = load_relay()
                uart.write_result_for[frame] = result
                self.assertIs(relay.uart_write_exact(frame), expected)
                self.assertEqual(uart.writes, [frame])

    def test_nonblocking_tcp_send_retries_eagain_without_losing_ack(self):
        relay, _uart = load_relay()
        connection = TemporarilyBlockedConnection(blocked_sends=2)

        relay.send_all(connection, b"ack")

        self.assertEqual(bytes(connection.sent), b"ack")

    def test_nonblocking_tcp_send_retries_none_without_losing_ack(self):
        relay, _uart = load_relay()
        connection = NoneThenWritableConnection(none_sends=2)

        relay.send_all(connection, b"ack")

        self.assertEqual(bytes(connection.sent), b"ack")

    def test_nonblocking_tcp_send_timeout_is_finite(self):
        relay, _uart = load_relay()
        connection = TemporarilyBlockedConnection(blocked_sends=1000)

        with self.assertRaisesRegex(OSError, "TCP send timed out"):
            relay.send_all(connection, b"ack")

    def test_init_still_allows_ready_path_when_uart_is_not_quiet(self):
        relay, _uart = load_relay()
        relay.drain_uart_until_quiet = lambda: False
        # Initialization itself must remain available for STOP diagnostics;
        # query_uart still rejects unsynchronized sensor responses separately.
        self.assertIsNone(relay.roomba_init())

    def test_brc_wake_is_disabled_by_default(self):
        relay, _uart = load_relay()

        relay.roomba_init()

        self.assertEqual(relay.brc.transitions, [])
        self.assertFalse(relay.brc_wake_sent)

    def test_enabled_brc_wake_pulses_only_once_per_boot(self):
        relay, _uart = load_relay()
        relay.BRC_WAKE_ENABLED = True

        relay.roomba_init()
        relay.roomba_init()

        self.assertEqual(relay.brc.transitions, [0, 1])
        self.assertTrue(relay.brc_wake_sent)

    def test_oi_mode_safe_and_other_defined_modes(self):
        for mode in range(4):
            with self.subTest(mode=mode):
                relay, uart = load_relay()
                uart.responses[bytes([relay.SENSORS, relay.SENSOR_OI_MODE])] = bytes(
                    [mode]
                )
                connection = FakeConnection(max_send=1)
                remaining, drives = relay.forward_valid_frames(
                    bytearray([relay.DIAGNOSTIC_MAGIC, relay.DIAGNOSTIC_OI_MODE]),
                    connection,
                )
                self.assertEqual(remaining, bytearray())
                self.assertEqual(drives, 0)
                self.assertEqual(
                    bytes(connection.sent),
                    bytes(
                        [relay.DIAGNOSTIC_MAGIC, relay.DIAGNOSTIC_OI_MODE_RESPONSE, mode]
                    ),
                )
                self.assertIn(relay.STOP, uart.writes)

    def test_oi_mode_missing_or_invalid_returns_error(self):
        for response in (b"", b"\x04"):
            with self.subTest(response=response):
                relay, uart = load_relay()
                uart.responses[bytes([relay.SENSORS, relay.SENSOR_OI_MODE])] = response
                connection = FakeConnection()
                _remaining, drives = relay.forward_valid_frames(
                    bytearray([relay.DIAGNOSTIC_MAGIC, relay.DIAGNOSTIC_OI_MODE]),
                    connection,
                )
                self.assertEqual(drives, 0)
                self.assertEqual(
                    bytes(connection.sent),
                    bytes([relay.DIAGNOSTIC_MAGIC, relay.DIAGNOSTIC_ERROR, 1]),
                )

    def test_live_oi_mode_query_does_not_send_stop_or_extend_drive_watchdog(self):
        relay, uart = load_relay()
        query = bytes([relay.SENSORS, relay.SENSOR_OI_MODE])
        uart.responses[query] = b"\x02"
        connection = FakeConnection(max_send=1)
        remaining, drives = relay.forward_valid_frames(
            bytearray(
                [relay.DIAGNOSTIC_MAGIC, relay.DIAGNOSTIC_OI_MODE_LIVE]
            ),
            connection,
        )
        self.assertEqual(remaining, bytearray())
        self.assertEqual(drives, 0)
        self.assertEqual(uart.writes, [relay.PAUSE_STREAM, query])
        self.assertEqual(
            bytes(connection.sent),
            bytes(
                [relay.DIAGNOSTIC_MAGIC, relay.DIAGNOSTIC_OI_MODE_RESPONSE, 2]
            ),
        )

    def test_stop_ack_reports_uart_and_robot_zero(self):
        relay, uart = load_relay()
        query = bytes(
            [
                relay.QUERY_LIST,
                2,
                relay.SENSOR_REQUESTED_RIGHT_VELOCITY,
                relay.SENSOR_REQUESTED_LEFT_VELOCITY,
            ]
        )
        uart.responses[query] = b"\x00\x00\x00\x00"
        connection = FakeConnection(max_send=2)
        _remaining, drives = relay.forward_valid_frames(
            bytearray([relay.DIAGNOSTIC_MAGIC, relay.DIAGNOSTIC_STOP]), connection
        )
        self.assertEqual(drives, 0)
        self.assertEqual(
            bytes(connection.sent),
            bytes([relay.DIAGNOSTIC_MAGIC, relay.DIAGNOSTIC_STOP_RESPONSE, 7])
            + b"\x00\x00\x00\x00",
        )
        self.assertEqual(uart.writes[:3], [relay.STOP, relay.PAUSE_STREAM, query])

    def test_stop_diagnostic_discards_stale_uart_bytes_before_query(self):
        relay, uart = load_relay()
        query = bytes(
            [
                relay.QUERY_LIST,
                2,
                relay.SENSOR_REQUESTED_RIGHT_VELOCITY,
                relay.SENSOR_REQUESTED_LEFT_VELOCITY,
            ]
        )
        # This exact stale suffix would otherwise decode left as 0x008e = 142.
        uart.read_buffer.extend(b"\x00\x00\x00\x8e")
        uart.responses[query] = b"\x00\x00\x00\x00"
        connection = FakeConnection()

        _remaining, drives = relay.forward_valid_frames(
            bytearray([relay.DIAGNOSTIC_MAGIC, relay.DIAGNOSTIC_STOP]), connection
        )

        self.assertEqual(drives, 0)
        self.assertEqual(
            bytes(connection.sent),
            bytes([relay.DIAGNOSTIC_MAGIC, relay.DIAGNOSTIC_STOP_RESPONSE, 7])
            + b"\x00\x00\x00\x00",
        )

    def test_uart_that_never_becomes_quiet_cannot_confirm_stop(self):
        relay, uart = load_relay()
        relay.drain_uart_until_quiet = lambda: False
        connection = FakeConnection()

        output = io.StringIO()
        with redirect_stdout(output):
            _remaining, drives = relay.forward_valid_frames(
                bytearray([relay.DIAGNOSTIC_MAGIC, relay.DIAGNOSTIC_STOP]),
                connection,
            )

        self.assertEqual(drives, 0)
        self.assertEqual(connection.sent[2], 0x03)
        self.assertEqual(bytes(connection.sent[3:]), b"\x7f\xff\x7f\xff")
        self.assertEqual(uart.writes, [relay.STOP, relay.PAUSE_STREAM])
        self.assertIn("UART diagnostic sync failed", output.getvalue())
        self.assertIn("command=95 02 29 2a", output.getvalue())

    def test_uart_drain_timeout_reports_bounded_discard_diagnostics(self):
        relay, uart = load_relay()
        uart.any = lambda: 1
        uart.read = lambda _size: b"\xaa"

        output = io.StringIO()
        with redirect_stdout(output):
            quiet = relay.drain_uart_until_quiet(quiet_ms=10, timeout_ms=100)

        self.assertFalse(quiet)
        self.assertRegex(output.getvalue(), r"discarded_count=[1-9][0-9]*")
        self.assertIn("tail=" + " ".join(["aa"] * 16), output.getvalue())

    def test_partial_stop_sensor_response_logs_raw_bytes_and_keeps_gate_closed(self):
        relay, uart = load_relay()
        query = bytes(
            [
                relay.QUERY_LIST,
                2,
                relay.SENSOR_REQUESTED_RIGHT_VELOCITY,
                relay.SENSOR_REQUESTED_LEFT_VELOCITY,
            ]
        )
        uart.responses[query] = b"\x00\x00\x8e"
        connection = FakeConnection()
        output = io.StringIO()

        with redirect_stdout(output):
            _remaining, drives = relay.forward_valid_frames(
                bytearray([relay.DIAGNOSTIC_MAGIC, relay.DIAGNOSTIC_STOP]),
                connection,
            )

        self.assertEqual(drives, 0)
        self.assertEqual(connection.sent[2], 0x03)
        self.assertEqual(bytes(connection.sent[3:]), b"\x7f\xff\x7f\xff")
        self.assertIn("received 3/4", output.getvalue())
        self.assertIn("raw=00 00 8e", output.getvalue())

    def test_stop_ack_does_not_claim_failed_uart_or_sensor_confirmation(self):
        relay, uart = load_relay()
        uart.partial_write_for = relay.STOP
        connection = FakeConnection()
        _remaining, drives = relay.forward_valid_frames(
            bytearray([relay.DIAGNOSTIC_MAGIC, relay.DIAGNOSTIC_STOP]), connection
        )
        self.assertEqual(drives, 0)
        self.assertEqual(connection.sent[2], relay.STOP_ACK_VALIDATED)

    def test_split_and_concatenated_diagnostics(self):
        relay, uart = load_relay()
        uart.responses[bytes([relay.SENSORS, relay.SENSOR_OI_MODE])] = b"\x02"
        query = bytes(
            [
                relay.QUERY_LIST,
                2,
                relay.SENSOR_REQUESTED_RIGHT_VELOCITY,
                relay.SENSOR_REQUESTED_LEFT_VELOCITY,
            ]
        )
        uart.responses[query] = b"\x00\x00\x00\x00"
        connection = FakeConnection()

        remaining, drives = relay.forward_valid_frames(
            bytearray([relay.DIAGNOSTIC_MAGIC]), connection
        )
        self.assertEqual(remaining, bytearray([relay.DIAGNOSTIC_MAGIC]))
        self.assertEqual(drives, 0)
        remaining.extend(
            bytes(
                [
                    relay.DIAGNOSTIC_STOP,
                    relay.DIAGNOSTIC_MAGIC,
                    relay.DIAGNOSTIC_OI_MODE,
                ]
            )
        )
        remaining, drives = relay.forward_valid_frames(remaining, connection)
        self.assertEqual(remaining, bytearray())
        self.assertEqual(drives, 0)
        self.assertEqual(len(connection.sent), 7 + 3)

    def test_nonzero_drive_is_rejected_but_legacy_stop_is_compatible(self):
        relay, uart = load_relay()
        connection = FakeConnection()
        remaining, drives = relay.forward_valid_frames(
            bytearray([145, 0, 50, 0, 50, 145, 0, 0, 0, 0]), connection
        )
        self.assertEqual(remaining, bytearray())
        self.assertEqual(drives, 0)
        self.assertEqual(uart.writes, [relay.STOP])

    def test_legacy_nonzero_is_rejected_even_when_hardware_lock_is_enabled(self):
        relay, uart = load_relay()
        relay.ALLOW_NONZERO_DRIVE = True
        connection = FakeConnection()
        output = io.StringIO()
        with redirect_stdout(output):
            remaining, drives = relay.forward_valid_frames(
                bytearray([145, 0, 20, 0, 20, 145, 0, 0, 0, 0]),
                connection,
            )
        self.assertEqual(remaining, bytearray())
        self.assertEqual(drives, 0)
        self.assertEqual(uart.writes, [relay.STOP])
        self.assertIn("drive rejected: generation required", output.getvalue())
        self.assertIn("normal STOP forwarded: at_ms=", output.getvalue())

    @staticmethod
    def arm_frame(relay, generation):
        return bytes([relay.DIAGNOSTIC_MAGIC, relay.CONTROL_ARM]) + relay.encoded_32(
            generation
        )

    @staticmethod
    def drive_frame(relay, generation, right=20, left=20, sequence=1):
        payload = (
            bytes([relay.DIAGNOSTIC_MAGIC, relay.SEQUENCED_DRIVE])
            + relay.encoded_32(generation)
            + relay.encoded_32(sequence)
            + bytes([relay.DRIVE_DIRECT])
            + relay.encoded_16(right)
            + relay.encoded_16(left)
        )
        return payload + relay.encoded_16(relay.crc16_ccitt(payload))

    def test_watchdog_latches_and_rejects_nonzero_until_new_arm(self):
        relay, uart = load_relay()
        relay.ALLOW_NONZERO_DRIVE = True
        connection = FakeConnection()
        output = io.StringIO()
        with redirect_stdout(output):
            _remaining, activity = relay.forward_valid_frames(
                bytearray(self.arm_frame(relay, 10) + self.drive_frame(relay, 10)),
                connection,
            )
            self.assertEqual(activity, 1)
            last_command = relay.time.ticks_ms()
            relay.time.now += relay.COMMAND_TIMEOUT_MS
            stopped = relay.enforce_command_watchdog(last_command, False)
            _remaining, activity = relay.forward_valid_frames(
                bytearray(self.drive_frame(relay, 10)), connection
            )
        self.assertTrue(stopped)
        self.assertEqual(activity, 0)
        self.assertEqual(relay.motion_latch.state, relay.MotionLatch.LATCHED_STOP)
        self.assertEqual(uart.writes[-1], relay.STOP)
        self.assertIn("watchdog STOP:", output.getvalue())
        self.assertIn("drive rejected: lock/generation/sequence", output.getvalue())

    def test_stale_generation_rejected_and_new_arm_generation_accepted(self):
        relay, uart = load_relay()
        relay.ALLOW_NONZERO_DRIVE = True
        connection = FakeConnection()
        output = io.StringIO()
        with redirect_stdout(output):
            relay.forward_valid_frames(
                bytearray(self.arm_frame(relay, 20)), connection
            )
            relay.motion_latch.latch_stop()
            relay.forward_valid_frames(
                bytearray(self.arm_frame(relay, 20)), connection
            )
            relay.forward_valid_frames(
                bytearray(self.arm_frame(relay, 21)), connection
            )
            _remaining, activity = relay.forward_valid_frames(
                bytearray(
                    self.drive_frame(relay, 20) + self.drive_frame(relay, 21)
                ),
                connection,
            )
        self.assertEqual(activity, 1)
        self.assertEqual(uart.writes, [b"\x91\x00\x14\x00\x14"])
        self.assertIn("ARM rejected: stale generation=20", output.getvalue())
        self.assertIn("ARM reserved: generation=21", output.getvalue())
        self.assertIn("ARM committed: generation=21", output.getvalue())

    def test_arm_commits_only_after_complete_ack_send(self):
        relay, _uart = load_relay()
        relay.ALLOW_NONZERO_DRIVE = True

        class ObservingConnection(FakeConnection):
            def send(self, data):
                self.state_during_send = relay.motion_latch.state
                return super().send(data)

        connection = ObservingConnection()
        relay.forward_valid_frames(
            bytearray(self.arm_frame(relay, 51)), connection
        )

        self.assertEqual(
            connection.state_during_send, relay.MotionLatch.ARM_PENDING
        )
        self.assertEqual(relay.motion_latch.state, relay.MotionLatch.ARMED)
        self.assertEqual(bytes(connection.sent)[:3], b"\xd7\x84\x01")

    def test_stop_during_arm_ack_prevents_commit(self):
        relay, uart = load_relay()
        relay.ALLOW_NONZERO_DRIVE = True

        class StopDuringSendConnection(FakeConnection):
            def send(self, data):
                relay.motion_latch.latch_stop()
                return super().send(data)

        with self.assertRaisesRegex(OSError, "ARM commit failed after ACK"):
            relay.forward_valid_frames(
                bytearray(self.arm_frame(relay, 54)), StopDuringSendConnection()
            )

        self.assertEqual(relay.motion_latch.state, relay.MotionLatch.LATCHED_STOP)
        self.assertEqual(relay.motion_latch.highest_generation, 54)
        self.assertIsNone(relay.motion_latch.generation)
        self.assertEqual(uart.writes[-1], relay.STOP)

    def test_arm_ack_send_failure_latches_stop_and_burns_generation(self):
        relay, uart = load_relay()
        relay.ALLOW_NONZERO_DRIVE = True
        connection = NoneThenWritableConnection(none_sends=1000)

        with self.assertRaisesRegex(OSError, "TCP send timed out"):
            relay.forward_valid_frames(
                bytearray(self.arm_frame(relay, 52)), connection
            )

        self.assertEqual(relay.motion_latch.state, relay.MotionLatch.LATCHED_STOP)
        self.assertEqual(relay.motion_latch.highest_generation, 52)
        self.assertIsNone(relay.motion_latch.generation)
        self.assertEqual(uart.writes[-1], relay.STOP)

        retry = FakeConnection()
        relay.forward_valid_frames(bytearray(self.arm_frame(relay, 52)), retry)
        self.assertEqual(retry.sent[2], relay.ARM_REJECTED_GENERATION)

    def test_arm_without_first_drive_expires_at_existing_500ms_limit(self):
        relay, uart = load_relay()
        relay.ALLOW_NONZERO_DRIVE = True
        relay.forward_valid_frames(
            bytearray(self.arm_frame(relay, 53)), FakeConnection()
        )
        armed_at = relay.motion_latch.armed_at_ms
        relay.time.now = armed_at + relay.COMMAND_TIMEOUT_MS

        self.assertTrue(relay.enforce_arm_watchdog())
        self.assertEqual(relay.motion_latch.state, relay.MotionLatch.LATCHED_STOP)
        self.assertEqual(uart.writes[-1], relay.STOP)

    def test_sequence_replay_and_bad_crc_are_rejected_without_uart_write(self):
        relay, uart = load_relay()
        relay.ALLOW_NONZERO_DRIVE = True
        connection = FakeConnection()
        relay.forward_valid_frames(bytearray(self.arm_frame(relay, 40)), connection)
        first = self.drive_frame(relay, 40, sequence=1)
        replay = self.drive_frame(relay, 40, sequence=1)
        corrupt = bytearray(self.drive_frame(relay, 40, sequence=2))
        corrupt[-1] ^= 0x01

        _remaining, activity = relay.forward_valid_frames(
            bytearray(first + replay + corrupt), connection
        )

        self.assertEqual(activity, 1)
        self.assertEqual(uart.writes, [b"\x91\x00\x14\x00\x14"])
        # ARM ACK (7 bytes), then one positive and two negative Drive ACKs.
        self.assertEqual(len(connection.sent), 7 + 13 * 3)

    def test_incomplete_drive_uart_results_return_negative_ack_and_latch_stop(self):
        frame = b"\x91\x00\x14\x00\x14"
        for result in (4, 0, None):
            with self.subTest(result=result):
                relay, uart = load_relay()
                relay.ALLOW_NONZERO_DRIVE = True
                connection = FakeConnection()
                relay.forward_valid_frames(
                    bytearray(self.arm_frame(relay, 41)), connection
                )
                drive = self.drive_frame(relay, 41, sequence=1)
                uart.write_result_for[frame] = result

                _remaining, activity = relay.forward_valid_frames(
                    bytearray(drive), connection
                )

                self.assertEqual(activity, 0)
                self.assertEqual(
                    relay.motion_latch.state, relay.MotionLatch.LATCHED_STOP
                )
                ack = bytes(connection.sent[-13:])
                self.assertEqual(ack[2], relay.DRIVE_ACK_VALIDATED)
                self.assertNotEqual(
                    ack[2] & relay.DRIVE_ACK_UART_WRITTEN,
                    relay.DRIVE_ACK_UART_WRITTEN,
                )
                self.assertEqual(uart.writes, [frame, relay.STOP])

    def test_stop_and_diagnostic_do_not_release_latched_stop(self):
        relay, uart = load_relay()
        relay.ALLOW_NONZERO_DRIVE = True
        relay.motion_latch.arm(30)
        relay.motion_latch.latch_stop()
        uart.responses[bytes([relay.SENSORS, relay.SENSOR_OI_MODE])] = b"\x02"
        connection = FakeConnection()
        relay.forward_valid_frames(
            bytearray(
                relay.STOP
                + bytes([relay.DIAGNOSTIC_MAGIC, relay.DIAGNOSTIC_OI_MODE])
            ),
            connection,
        )
        self.assertEqual(relay.motion_latch.state, relay.MotionLatch.LATCHED_STOP)

    def test_500ms_watchdog_writes_exact_stop_once_and_logs_timing(self):
        relay, uart = load_relay()
        last_command = relay.time.ticks_ms()
        relay.time.now += relay.COMMAND_TIMEOUT_MS
        output = io.StringIO()
        with redirect_stdout(output):
            stopped = relay.enforce_command_watchdog(last_command, False)
            stopped = relay.enforce_command_watchdog(last_command, stopped)
        self.assertTrue(stopped)
        self.assertEqual(uart.writes, [relay.STOP])
        self.assertIn("watchdog STOP: fired_at_ms=", output.getvalue())
        self.assertIn("elapsed_ms=", output.getvalue())
        self.assertIn("uart_written=True", output.getvalue())


if __name__ == "__main__":
    unittest.main()

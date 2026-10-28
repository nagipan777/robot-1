import socket
import struct
import threading
import time
import unittest

from simulation.roomba_tcp_transport import (
    CommandState,
    DriveSessionLimiter,
    DriveDeadlineExpired,
    DIAGNOSTIC_MAGIC,
    DIAGNOSTIC_OI_MODE,
    DIAGNOSTIC_OI_MODE_LIVE,
    DIAGNOSTIC_OI_MODE_RESPONSE,
    DIAGNOSTIC_STOP,
    DIAGNOSTIC_STOP_RESPONSE,
    CONTROL_ARM,
    CONTROL_ARM_RESPONSE,
    DRIVE_ACK_COMPLETE,
    SEQUENCED_DRIVE,
    SEQUENCED_DRIVE_RESPONSE,
    GENERATION_DRIVE,
    RELAY_READY,
    STOP_PACKET,
    RoombaDiagnosticError,
    RoombaTcpTransport,
    SafeModeGate,
    STOP_ACK_COMPLETE,
    WatchdogTestCommandState,
    drive_direct_packet,
    with_crc,
)


class LocalRelayServer:
    def __init__(self, handlers):
        self.handlers = handlers
        self.received = []
        self.error = None
        self.listener = socket.socket()
        self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listener.bind(("127.0.0.1", 0))
        self.listener.listen(len(handlers))
        self.host, self.port = self.listener.getsockname()
        self.thread = threading.Thread(target=self.run, daemon=True)

    def __enter__(self):
        self.thread.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.listener.close()
        self.thread.join(timeout=2.0)
        if exc_type is None and self.error is not None:
            raise self.error

    def run(self):
        try:
            for handler in self.handlers:
                connection, _ = self.listener.accept()
                with connection:
                    handler(connection, self.received)
        except BaseException as error:
            self.error = error


def receive_exact(connection, size):
    data = bytearray()
    while len(data) < size:
        chunk = connection.recv(size - len(data))
        if not chunk:
            break
        data.extend(chunk)
    return bytes(data)


def ready_handler(connection, received):
    connection.sendall(RELAY_READY)
    received.append(receive_exact(connection, len(STOP_PACKET)))


class RoombaTcpTransportTests(unittest.TestCase):
    def test_drive_trace_separates_send_receive_and_validation(self):
        generation = 9
        sequence = 1
        response = with_crc(
            bytes(
                [
                    DIAGNOSTIC_MAGIC,
                    SEQUENCED_DRIVE_RESPONSE,
                    DRIVE_ACK_COMPLETE,
                ]
            )
            + struct.pack(">II", generation, sequence)
        )

        class FakeSocket:
            def __init__(self):
                self.sent = []
                self.response = bytearray(response)

            def sendall(self, packet):
                self.sent.append(packet)

            def recv(self, size):
                chunk = bytes(self.response[:size])
                del self.response[:size]
                return chunk

        events = []
        transport = RoombaTcpTransport(
            "127.0.0.1", 9000, trace=lambda stage, **fields: events.append(
                (stage, fields)
            )
        )
        transport.socket = FakeSocket()
        transport.armed_generation = generation

        self.assertEqual(
            transport.send_twist(
                0.01, 0.0, track_width_m=0.235, max_speed_mm_s=20
            ),
            sequence,
        )
        self.assertEqual(
            [stage for stage, _ in events],
            [
                "transport_send_start",
                "tcp_drive_sent",
                "tcp_drive_ack_wait_start",
                "tcp_drive_ack_received",
                "tcp_drive_ack_validated",
            ],
        )
        self.assertTrue(all(fields["generation"] == generation for _, fields in events))

    def test_deadline_interrupts_delayed_drive_ack_and_poisoned_session(self):
        class DelayedSocket:
            def __init__(self, delay_s):
                self.delay_s = delay_s
                self.ready_at = None
                self.timeout = 2.0
                self.sent = []
                self.closed = False

            def sendall(self, packet):
                self.sent.append(packet)
                if self.ready_at is None:
                    self.ready_at = time.monotonic() + self.delay_s

            def gettimeout(self):
                return self.timeout

            def settimeout(self, timeout):
                self.timeout = timeout

            def recv(self, _size):
                remaining = self.ready_at - time.monotonic()
                if remaining > self.timeout:
                    time.sleep(self.timeout)
                    raise socket.timeout()
                time.sleep(max(0.0, remaining))
                raise AssertionError("delayed ACK must not outlive the deadline")

            def close(self):
                self.closed = True

        for delay_s in (0.050, 0.181, 0.300):
            with self.subTest(delay_s=delay_s):
                events = []
                connection = DelayedSocket(delay_s)
                transport = RoombaTcpTransport(
                    "unused",
                    9000,
                    trace=lambda stage, **fields: events.append((stage, fields)),
                )
                transport.socket = connection
                transport.armed_generation = 41
                deadline = time.monotonic() + 0.025
                with self.assertRaises(DriveDeadlineExpired) as caught:
                    transport.send_twist(
                        0.02,
                        0.0,
                        track_width_m=0.235,
                        max_speed_mm_s=20,
                        deadline_monotonic=deadline,
                    )
                message = str(caught.exception)
                self.assertIn("deadline=", message)
                self.assertIn("stop_started=", message)
                self.assertIn("overdue_s=", message)
                self.assertIn("stop_send_elapsed_s=", message)

                stages = [stage for stage, _ in events]
                self.assertIn("tcp_deadline_stop_begin", stages)
                self.assertIn("tcp_deadline_stop_return", stages)
                stop_event = next(
                    fields
                    for stage, fields in events
                    if stage == "tcp_deadline_stop_begin"
                )
                self.assertLess(stop_event["stop_started"] - deadline, 0.020)
                self.assertEqual(connection.sent[-1], STOP_PACKET)
                self.assertEqual(len(connection.sent), 2)  # one Drive, one STOP
                self.assertTrue(connection.closed)
                self.assertFalse(transport.connected)
                self.assertIsNone(transport.armed_generation)
                self.assertEqual(transport.drive_sequence, 0)

    def test_deadline_interrupts_live_safe_query_before_drive(self):
        class SilentSocket:
            def __init__(self):
                self.timeout = 2.0
                self.sent = []
                self.closed = False

            def sendall(self, packet):
                self.sent.append(packet)

            def gettimeout(self):
                return self.timeout

            def settimeout(self, timeout):
                self.timeout = timeout

            def recv(self, _size):
                time.sleep(self.timeout)
                raise socket.timeout()

            def close(self):
                self.closed = True

        connection = SilentSocket()
        transport = RoombaTcpTransport("unused", 9000)
        transport.socket = connection
        transport.armed_generation = 99
        with self.assertRaises(DriveDeadlineExpired):
            transport.query_oi_mode(
                live=True, deadline_monotonic=time.monotonic() + 0.025
            )
        self.assertEqual(
            connection.sent,
            [bytes([DIAGNOSTIC_MAGIC, DIAGNOSTIC_OI_MODE_LIVE]), STOP_PACKET],
        )
        self.assertTrue(connection.closed)
        self.assertFalse(transport.connected)
        self.assertIsNone(transport.armed_generation)

    def test_deadline_closes_after_partial_ack_without_response_reuse(self):
        class PartialSocket:
            def __init__(self):
                self.timeout = 2.0
                self.sent = []
                self.partial_sent = False
                self.closed = False

            def sendall(self, packet):
                self.sent.append(packet)

            def gettimeout(self):
                return self.timeout

            def settimeout(self, timeout):
                self.timeout = timeout

            def recv(self, _size):
                if not self.partial_sent:
                    self.partial_sent = True
                    return bytes([DIAGNOSTIC_MAGIC])
                time.sleep(self.timeout)
                raise socket.timeout()

            def close(self):
                self.closed = True

        connection = PartialSocket()
        transport = RoombaTcpTransport("unused", 9000)
        transport.socket = connection
        transport.armed_generation = 100
        with self.assertRaisesRegex(DriveDeadlineExpired, r"received 1/13 bytes"):
            transport.send_twist(
                0.02,
                0.0,
                track_width_m=0.235,
                max_speed_mm_s=20,
                deadline_monotonic=time.monotonic() + 0.025,
            )
        self.assertEqual(connection.sent[-1], STOP_PACKET)
        self.assertTrue(connection.closed)
        self.assertFalse(transport.connected)

    def test_bounded_drive_session_expires_and_requires_new_arm(self):
        now = [10.0]
        limiter = DriveSessionLimiter(0.4, lambda: now[0])
        limiter.arm()
        self.assertEqual(limiter.observe_nonzero(), "started")
        now[0] += 0.399
        self.assertEqual(limiter.observe_nonzero(), "allowed")
        self.assertFalse(limiter.poll())
        now[0] += 0.001
        self.assertTrue(limiter.poll())
        self.assertEqual(limiter.observe_nonzero(), "latched")

        limiter.arm()
        self.assertEqual(limiter.observe_nonzero(), "started")

    def test_dispatch_permission_boundary_is_strict(self):
        now = [30.0]
        limiter = DriveSessionLimiter(0.4, lambda: now[0])
        limiter.arm()
        self.assertEqual(limiter.observe_nonzero(), "started")
        self.assertEqual(limiter.deadline, 30.4)
        now[0] = 30.399999
        self.assertTrue(limiter.permits_dispatch())
        now[0] = 30.4
        self.assertFalse(limiter.permits_dispatch())
        self.assertTrue(limiter.latched)

    def test_transport_rejects_expired_drive_before_socket_send(self):
        class NoSendSocket:
            def sendall(self, _packet):
                raise AssertionError("expired Drive reached socket")

        transport = RoombaTcpTransport("unused", 9000)
        transport.socket = NoSendSocket()
        transport.armed_generation = 123
        with self.assertRaises(DriveDeadlineExpired):
            transport.send_twist(
                0.02,
                0.0,
                track_width_m=0.235,
                max_speed_mm_s=20,
                deadline_monotonic=time.monotonic() - 0.001,
            )
        self.assertEqual(transport.drive_sequence, 0)

    def test_bounded_drive_session_rejects_late_callback_without_timer_poll(self):
        now = [20.0]
        limiter = DriveSessionLimiter(0.4, lambda: now[0])
        limiter.arm()
        self.assertEqual(limiter.observe_nonzero(), "started")

        # Repeated Navigation messages do not refresh the fixed deadline.
        now[0] += 0.2
        self.assertEqual(limiter.observe_nonzero(), "allowed")
        now[0] += 0.201
        self.assertEqual(limiter.observe_nonzero(), "expired")
        self.assertEqual(limiter.observe_nonzero(), "latched")

    def test_short_e_ros_delay_or_silence_cannot_extend_fixed_deadline(self):
        now = [100.0]
        limiter = DriveSessionLimiter(0.4, lambda: now[0])
        limiter.arm()
        self.assertEqual(limiter.observe_nonzero(), "started")
        started_at = limiter.started_at

        # Repeated fresh ROS commands do not slide the one-shot deadline.
        for offset in (0.05, 0.20, 0.399):
            now[0] = started_at + offset
            self.assertEqual(limiter.observe_nonzero(), "allowed")
            self.assertEqual(limiter.started_at, started_at)

        # Model a stalled ROS executor / communication gap.  On resumption,
        # the late command is rejected before it can refresh the deadline.
        now[0] = started_at + 1.65
        self.assertEqual(limiter.observe_nonzero(), "expired")
        self.assertTrue(limiter.latched)
        self.assertEqual(limiter.started_at, started_at)
        self.assertEqual(limiter.observe_nonzero(), "latched")

        # If the host stops dispatching completely, the independent Pico
        # watchdog bound remains 0.4 + 0.5 = 0.9 seconds from first Drive.
        pico_watchdog_s = 0.5
        self.assertLessEqual(limiter.duration_s + pico_watchdog_s, 1.0)

    def test_short_e_stop_latch_rejects_delayed_old_command_until_new_arm(self):
        now = [200.0]
        limiter = DriveSessionLimiter(0.4, lambda: now[0])
        limiter.arm()
        self.assertEqual(limiter.observe_nonzero(), "started")

        # A normal or deadline STOP latches the session.  An old Twist that
        # arrives later cannot restart it, regardless of how late it arrives.
        limiter.latch_stop()
        for delay in (0.001, 0.76, 1.65, 10.0):
            now[0] += delay
            self.assertEqual(limiter.observe_nonzero(), "latched")

        # Only a separate explicit ARM creates a new one-shot session.
        limiter.arm()
        self.assertEqual(limiter.observe_nonzero(), "started")

    def test_bounded_drive_stop_latches_and_invalid_limits_are_rejected(self):
        now = [1.0]
        limiter = DriveSessionLimiter(1.0, lambda: now[0])
        limiter.arm()
        self.assertEqual(limiter.observe_nonzero(), "started")
        limiter.latch_stop()
        self.assertEqual(limiter.observe_nonzero(), "latched")

        for duration in (-0.001, 1.001, float("nan"), True):
            with self.subTest(duration=duration):
                with self.assertRaises(ValueError):
                    DriveSessionLimiter(duration)

    def test_disabled_drive_session_limiter_preserves_existing_paths(self):
        now = [1.0]
        limiter = DriveSessionLimiter(0.0, lambda: now[0])
        limiter.arm()
        self.assertEqual(limiter.observe_nonzero(), "allowed")
        now[0] += 100.0
        self.assertEqual(limiter.observe_nonzero(), "allowed")
        self.assertFalse(limiter.poll())

    def test_safe_gate_requires_complete_stop_ack_and_safe_mode(self):
        gate = SafeModeGate()
        self.assertTrue(gate.permits(0.0, 0.0))
        self.assertFalse(gate.permits(0.01, 0.0))

        gate.establish(STOP_ACK_COMPLETE, 0, 0, 2)
        self.assertTrue(gate.permits(0.01, 0.0))

        gate.reset()
        self.assertFalse(gate.permits(0.01, 0.0))

    def test_safe_gate_rejects_incomplete_or_invalid_confirmation(self):
        cases = (
            (0x03, 0, 0, 2),
            (STOP_ACK_COMPLETE, 1, 0, 2),
            (STOP_ACK_COMPLETE, 0, -1, 2),
            (STOP_ACK_COMPLETE, 0, 0, 0),
            (STOP_ACK_COMPLETE, 0, 0, 1),
            (STOP_ACK_COMPLETE, 0, 0, 3),
        )
        for values in cases:
            with self.subTest(values=values):
                gate = SafeModeGate()
                with self.assertRaises(RoombaDiagnosticError):
                    gate.establish(*values)
                self.assertFalse(gate.authorized)
                self.assertFalse(gate.permits(0.01, 0.0))
                self.assertTrue(gate.permits(0.0, 0.0))

    def test_safe_gate_revokes_permission_when_safe_mode_is_lost(self):
        for mode in (0, 1, 3):
            with self.subTest(mode=mode):
                gate = SafeModeGate()
                gate.establish(STOP_ACK_COMPLETE, 0, 0, 2)
                with self.assertRaises(RoombaDiagnosticError):
                    gate.observe_mode(mode)
                self.assertFalse(gate.authorized)

    def test_fragmented_ready_and_initial_stop(self):
        def fragmented(connection, received):
            for fragment in (b"R", b"EA", b"DY", b"\n"):
                connection.sendall(fragment)
            received.append(receive_exact(connection, len(STOP_PACKET)))

        with LocalRelayServer([fragmented]) as server:
            transport = RoombaTcpTransport(server.host, server.port)
            transport.connect()
            transport.close(send_stop=False)
        self.assertEqual(server.received, [STOP_PACKET])

    def test_invalid_ready_is_rejected(self):
        def invalid(connection, _received):
            connection.sendall(b"BROKEN")

        with LocalRelayServer([invalid]) as server:
            transport = RoombaTcpTransport(server.host, server.port)
            with self.assertRaises(ConnectionError):
                transport.connect()
            self.assertFalse(transport.connected)

    def test_ready_zero_bytes_timeout_reports_exact_progress(self):
        def silent(connection, _received):
            time.sleep(0.1)

        with LocalRelayServer([silent]) as server:
            transport = RoombaTcpTransport(
                server.host, server.port, connect_timeout=0.05
            )
            with self.assertRaisesRegex(
                RoombaDiagnosticError, r"relay READY timed out: received 0/6 bytes"
            ):
                transport.connect()
            self.assertFalse(transport.connected)

    def test_ready_partial_then_close_reports_exact_progress(self):
        def partial(connection, _received):
            connection.sendall(b"REA")

        with LocalRelayServer([partial]) as server:
            transport = RoombaTcpTransport(server.host, server.port)
            with self.assertRaisesRegex(
                ConnectionError, r"relay READY \(3/6 bytes; raw=524541\)"
            ):
                transport.connect()
            self.assertFalse(transport.connected)

    def test_twist_conversion_limit_and_right_left_order(self):
        expected_drive = drive_direct_packet(31, 50)
        expected = with_crc(
            bytes([DIAGNOSTIC_MAGIC, SEQUENCED_DRIVE])
            + struct.pack(">II", 1, 1)
            + expected_drive
        )

        def capture(connection, received):
            connection.sendall(RELAY_READY)
            received.append(receive_exact(connection, len(STOP_PACKET)))
            received.append(receive_exact(connection, len(expected)))
            connection.sendall(
                with_crc(
                    bytes(
                        [
                            DIAGNOSTIC_MAGIC,
                            SEQUENCED_DRIVE_RESPONSE,
                            DRIVE_ACK_COMPLETE,
                        ]
                    )
                    + struct.pack(">II", 1, 1)
                )
            )

        with LocalRelayServer([capture]) as server:
            transport = RoombaTcpTransport(server.host, server.port)
            transport.connect()
            transport.armed_generation = 1
            transport.send_twist(
                0.1, 0.2, track_width_m=0.235, max_speed_mm_s=50
            )
            transport.close(send_stop=False)
        self.assertEqual(server.received, [STOP_PACKET, expected])
        self.assertEqual(struct.unpack(">Bhh", expected_drive), (145, 50, 31))

    def test_nonzero_drive_requires_arm_generation(self):
        transport = RoombaTcpTransport("127.0.0.1", 9000)
        transport.socket = object()
        with self.assertRaisesRegex(RoombaDiagnosticError, "ARM is required"):
            transport.send_twist(
                0.02, 0.0, track_width_m=0.235, max_speed_mm_s=20
            )

    def test_arm_fragmented_ack_sets_generation_and_tags_drive(self):
        generation = 1234
        expected_arm = (
            bytes([DIAGNOSTIC_MAGIC, CONTROL_ARM])
            + struct.pack(">I", generation)
        )
        expected_drive = with_crc(
            bytes([DIAGNOSTIC_MAGIC, SEQUENCED_DRIVE])
            + struct.pack(">II", generation, 1)
            + drive_direct_packet(20, 20)
        )

        def arm_and_drive(connection, received):
            connection.sendall(RELAY_READY)
            received.append(receive_exact(connection, len(STOP_PACKET)))
            received.append(receive_exact(connection, len(expected_arm)))
            response = (
                bytes([DIAGNOSTIC_MAGIC, CONTROL_ARM_RESPONSE, 1])
                + struct.pack(">I", generation)
            )
            connection.sendall(response[:2])
            connection.sendall(response[2:])
            received.append(receive_exact(connection, len(expected_drive)))
            connection.sendall(
                with_crc(
                    bytes(
                        [
                            DIAGNOSTIC_MAGIC,
                            SEQUENCED_DRIVE_RESPONSE,
                            DRIVE_ACK_COMPLETE,
                        ]
                    )
                    + struct.pack(">II", generation, 1)
                )
            )

        with LocalRelayServer([arm_and_drive]) as server:
            transport = RoombaTcpTransport(server.host, server.port)
            transport.connect()
            self.assertEqual(transport.arm(generation), generation)
            transport.send_twist(
                0.02, 0.0, track_width_m=0.235, max_speed_mm_s=20
            )
            transport.close(send_stop=False)
        self.assertEqual(server.received, [STOP_PACKET, expected_arm, expected_drive])

    def test_arm_partial_ack_close_reports_three_of_seven_and_revokes(self):
        generation = 1235

        def partial_ack(connection, received):
            connection.sendall(RELAY_READY)
            received.append(receive_exact(connection, len(STOP_PACKET)))
            received.append(receive_exact(connection, 6))
            connection.sendall(bytes([DIAGNOSTIC_MAGIC, CONTROL_ARM_RESPONSE, 1]))

        with LocalRelayServer([partial_ack]) as server:
            transport = RoombaTcpTransport(server.host, server.port)
            transport.connect()
            with self.assertRaisesRegex(
                ConnectionError, r"ARM response \(3/7 bytes; raw=d78401\)"
            ):
                transport.arm(generation)
            self.assertIsNone(transport.armed_generation)
            transport.close(send_stop=False)

    def test_old_generation_ack_is_rejected_without_authorization(self):
        requested = 1236

        def old_ack(connection, received):
            connection.sendall(RELAY_READY)
            received.append(receive_exact(connection, len(STOP_PACKET)))
            received.append(receive_exact(connection, 6))
            connection.sendall(
                bytes([DIAGNOSTIC_MAGIC, CONTROL_ARM_RESPONSE, 1])
                + struct.pack(">I", requested - 1)
            )

        with LocalRelayServer([old_ack]) as server:
            transport = RoombaTcpTransport(server.host, server.port)
            transport.connect()
            with self.assertRaisesRegex(
                RoombaDiagnosticError, "highest_generation=1235"
            ):
                transport.arm(requested)
            self.assertIsNone(transport.armed_generation)
            transport.close(send_stop=False)

    def test_drive_ack_failure_is_rejected_and_sequence_is_not_advanced(self):
        generation = 7

        def reject_drive(connection, received):
            connection.sendall(RELAY_READY)
            received.append(receive_exact(connection, len(STOP_PACKET)))
            received.append(receive_exact(connection, 17))
            connection.sendall(
                with_crc(
                    bytes([DIAGNOSTIC_MAGIC, SEQUENCED_DRIVE_RESPONSE, 1])
                    + struct.pack(">II", generation, 1)
                )
            )

        with LocalRelayServer([reject_drive]) as server:
            transport = RoombaTcpTransport(server.host, server.port)
            transport.connect()
            transport.armed_generation = generation
            with self.assertRaisesRegex(RoombaDiagnosticError, "Drive rejected"):
                transport.send_twist(
                    0.02, 0.0, track_width_m=0.235, max_speed_mm_s=20
                )
            self.assertEqual(transport.drive_sequence, 0)
            transport.close(send_stop=False)

    def test_reconnect_discards_host_arm_generation(self):
        transport = RoombaTcpTransport("127.0.0.1", 9000)
        transport.armed_generation = 99
        transport.close(send_stop=False)
        self.assertIsNone(transport.armed_generation)

    def test_arm_timeout_revokes_prior_generation_without_drive_retry(self):
        class TimeoutSocket:
            def __init__(self):
                self.sent = []

            def sendall(self, packet):
                self.sent.append(packet)

            def recv(self, _size):
                raise socket.timeout("timed out")

        events = []
        connection = TimeoutSocket()
        transport = RoombaTcpTransport(
            "127.0.0.1",
            9000,
            trace=lambda stage, **fields: events.append((stage, fields)),
        )
        transport.socket = connection
        transport.armed_generation = 88
        transport.drive_sequence = 4

        with self.assertRaisesRegex(
            RoombaDiagnosticError,
            r"ARM response timed out: received 0/7 bytes",
        ):
            transport.arm(89)

        self.assertIsNone(transport.armed_generation)
        self.assertEqual(transport.drive_sequence, 0)
        self.assertEqual(len(connection.sent), 1)
        self.assertEqual(connection.sent[0][:2], bytes([DIAGNOSTIC_MAGIC, CONTROL_ARM]))
        self.assertIn("tcp_arm_response_failed", [stage for stage, _ in events])

    def test_timeout_and_stop_only_gate(self):
        now = [10.0]
        state = CommandState(True, 0.5, lambda: now[0])
        state.update(0.03, 0.0)
        self.assertEqual(state.current(), (0.03, 0.0))
        now[0] += 0.501
        self.assertIsNone(state.current())
        state.update(0.02, 0.0)
        self.assertEqual(state.current(), (0.02, 0.0))

        disabled = CommandState(False, 0.5, lambda: now[0])
        disabled.update(0.03, 0.0)
        self.assertIsNone(disabled.current())

    def test_watchdog_mode_forwards_fresh_commands_once_then_stays_silent(self):
        now = [10.0]
        state = WatchdogTestCommandState(0.75, lambda: now[0])
        state.update(0.02, 0.0)
        self.assertEqual(state.poll(), ("command", (0.02, 0.0)))
        self.assertEqual(state.poll(), ("silence", None))
        now[0] += 0.10
        self.assertEqual(state.poll(), ("silence_start", None))
        now[0] += 0.399
        self.assertEqual(state.poll(), ("silence", None))
        now[0] += 0.251
        self.assertEqual(state.poll(), ("fallback_stop", None))
        self.assertEqual(state.poll(), ("silence", None))

    def test_watchdog_mode_new_command_cancels_old_silence_deadline(self):
        now = [20.0]
        state = WatchdogTestCommandState(0.75, lambda: now[0])
        state.update(0.02, 0.0)
        state.poll()
        now[0] += 0.6
        state.update(0.01, 0.0)
        self.assertEqual(state.poll(), ("command", (0.01, 0.0)))
        now[0] += 0.5
        self.assertNotEqual(state.poll()[0], "fallback_stop")

    def test_stop_only_nonzero_twist_sends_only_stop_packets(self):
        def capture(connection, received):
            connection.sendall(RELAY_READY)
            received.append(receive_exact(connection, len(STOP_PACKET) * 2))

        with LocalRelayServer([capture]) as server:
            transport = RoombaTcpTransport(server.host, server.port)
            state = CommandState(False, 0.5, lambda: 1.0)
            transport.connect()
            state.update(0.5, 2.0)
            command = state.current()
            if command is None:
                transport.send_stop()
            else:
                transport.send_twist(
                    command[0],
                    command[1],
                    track_width_m=0.235,
                    max_speed_mm_s=50,
                )
            transport.close(send_stop=False)

        self.assertEqual(server.received, [STOP_PACKET + STOP_PACKET])
        for offset in range(0, len(server.received[0]), len(STOP_PACKET)):
            self.assertEqual(
                struct.unpack(">Bhh", server.received[0][offset : offset + 5]),
                (145, 0, 0),
            )

    def test_reconnect_sends_stop_and_requires_new_command(self):
        with LocalRelayServer([ready_handler, ready_handler]) as server:
            transport = RoombaTcpTransport(server.host, server.port)
            state = CommandState(True, 0.5, lambda: 1.0)
            transport.connect()
            state.update(0.03, 0.0)
            transport.close(send_stop=False)

            # This is the reconnect invariant used by the ROS node.
            state.clear()
            transport.connect()
            self.assertIsNone(state.current())
            transport.close(send_stop=False)
        self.assertEqual(server.received, [STOP_PACKET, STOP_PACKET])

    def test_send_failure_disconnects_cleanly(self):
        class FailingSocket:
            def sendall(self, _packet):
                raise BrokenPipeError("test disconnect")

            def close(self):
                pass

        transport = RoombaTcpTransport("127.0.0.1", 9000)
        transport.socket = FailingSocket()
        with self.assertRaises(BrokenPipeError):
            transport.send_stop()
        transport.close(send_stop=False)
        self.assertFalse(transport.connected)

    def test_close_sends_stop(self):
        def capture_both_stops(connection, received):
            connection.sendall(RELAY_READY)
            received.append(receive_exact(connection, len(STOP_PACKET) * 2))

        with LocalRelayServer([capture_both_stops]) as server:
            transport = RoombaTcpTransport(server.host, server.port)
            transport.connect()
            transport.close(send_stop=True)
        self.assertEqual(server.received, [STOP_PACKET + STOP_PACKET])

    def test_stop_ack_confirms_all_stages_and_zero_requested_velocity(self):
        def diagnostic(connection, received):
            connection.sendall(RELAY_READY)
            received.append(receive_exact(connection, 5))
            received.append(receive_exact(connection, 2))
            response = bytes([DIAGNOSTIC_MAGIC, DIAGNOSTIC_STOP_RESPONSE, 7])
            connection.sendall(response[:2])
            connection.sendall(response[2:] + b"\x00\x00\x00\x00")

        with LocalRelayServer([diagnostic]) as server:
            transport = RoombaTcpTransport(server.host, server.port)
            transport.connect()
            self.assertEqual(transport.verify_stop(), (7, 0, 0))
            transport.close(send_stop=False)
        self.assertEqual(
            server.received,
            [STOP_PACKET, bytes([DIAGNOSTIC_MAGIC, DIAGNOSTIC_STOP])],
        )

    def test_stop_ack_failure_is_rejected(self):
        for status, right, left in ((1, 0, 0), (3, 32767, 32767), (7, 1, 0)):
            with self.subTest(status=status, right=right, left=left):
                def diagnostic(connection, _received):
                    connection.sendall(RELAY_READY)
                    receive_exact(connection, 5)
                    receive_exact(connection, 2)
                    connection.sendall(
                        bytes([DIAGNOSTIC_MAGIC, DIAGNOSTIC_STOP_RESPONSE, status])
                        + struct.pack(">hh", right, left)
                    )

                with LocalRelayServer([diagnostic]) as server:
                    transport = RoombaTcpTransport(server.host, server.port)
                    transport.connect()
                    with self.assertRaises(RoombaDiagnosticError):
                        transport.verify_stop()
                    transport.close(send_stop=False)

    def test_stop_ack_failure_reports_fixed_response_bytes(self):
        def diagnostic(connection, _received):
            connection.sendall(RELAY_READY)
            receive_exact(connection, 5)
            receive_exact(connection, 2)
            connection.sendall(b"\xd7\x81\x03\x7f\xff\x7f\xff")

        with LocalRelayServer([diagnostic]) as server:
            transport = RoombaTcpTransport(server.host, server.port)
            transport.connect()
            with self.assertRaisesRegex(
                RoombaDiagnosticError, r"raw=d781037fff7fff"
            ):
                transport.verify_stop()
            transport.close(send_stop=False)

    def test_oi_mode_values_zero_through_three(self):
        for mode in range(4):
            with self.subTest(mode=mode):
                def diagnostic(connection, _received):
                    connection.sendall(RELAY_READY)
                    receive_exact(connection, 5)
                    request = receive_exact(connection, 2)
                    self.assertEqual(
                        request, bytes([DIAGNOSTIC_MAGIC, DIAGNOSTIC_OI_MODE])
                    )
                    response = bytes(
                        [DIAGNOSTIC_MAGIC, DIAGNOSTIC_OI_MODE_RESPONSE, mode]
                    )
                    connection.sendall(response[:1])
                    connection.sendall(response[1:])

                with LocalRelayServer([diagnostic]) as server:
                    transport = RoombaTcpTransport(server.host, server.port)
                    transport.connect()
                    self.assertEqual(transport.query_oi_mode(), mode)
                    transport.close(send_stop=False)

    def test_live_oi_mode_query_uses_fixed_command_and_fragmented_response(self):
        def diagnostic(connection, received):
            connection.sendall(RELAY_READY)
            received.append(receive_exact(connection, 5))
            received.append(receive_exact(connection, 2))
            response = bytes(
                [DIAGNOSTIC_MAGIC, DIAGNOSTIC_OI_MODE_RESPONSE, 2]
            )
            connection.sendall(response[:1])
            connection.sendall(response[1:2])
            connection.sendall(response[2:])

        with LocalRelayServer([diagnostic]) as server:
            transport = RoombaTcpTransport(server.host, server.port)
            transport.connect()
            self.assertEqual(transport.query_oi_mode(live=True), 2)
            transport.close(send_stop=False)
        self.assertEqual(
            server.received,
            [
                STOP_PACKET,
                bytes([DIAGNOSTIC_MAGIC, DIAGNOSTIC_OI_MODE_LIVE]),
            ],
        )

    def test_live_oi_mode_timeout_identifies_wait_and_partial_raw_bytes(self):
        class PartialTimeoutSocket:
            def __init__(self):
                self.calls = 0

            def sendall(self, _packet):
                pass

            def recv(self, _size):
                self.calls += 1
                if self.calls == 1:
                    return bytes([DIAGNOSTIC_MAGIC])
                raise socket.timeout("timed out")

        transport = RoombaTcpTransport("127.0.0.1", 9000)
        transport.socket = PartialTimeoutSocket()
        with self.assertRaisesRegex(
            RoombaDiagnosticError,
            r"live OI mode diagnostic timed out: received 1/3 bytes; raw=d7",
        ):
            transport.query_oi_mode(live=True)

    def test_invalid_or_missing_oi_mode_response_is_rejected(self):
        def invalid(connection, _received):
            connection.sendall(RELAY_READY)
            receive_exact(connection, 5)
            receive_exact(connection, 2)
            connection.sendall(
                bytes([DIAGNOSTIC_MAGIC, DIAGNOSTIC_OI_MODE_RESPONSE, 4])
            )

        def missing(connection, _received):
            connection.sendall(RELAY_READY)
            receive_exact(connection, 5)
            receive_exact(connection, 2)

        for handler in (invalid, missing):
            with self.subTest(handler=handler.__name__):
                with LocalRelayServer([handler]) as server:
                    transport = RoombaTcpTransport(server.host, server.port)
                    transport.connect()
                    with self.assertRaises((ConnectionError, RoombaDiagnosticError)):
                        transport.query_oi_mode()
                    transport.close(send_stop=False)


if __name__ == "__main__":
    unittest.main()

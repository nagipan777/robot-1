import pathlib
import importlib.util
import sys
import unittest

from pico.dry_run_protocol import (
    DryRunReceiverState,
    ProtocolError,
    crc16_ccitt,
    decode_ack,
    decode_command,
    decode_ready,
    encode_ack,
    encode_command,
    encode_ready,
)


ROOT = pathlib.Path(__file__).resolve().parents[1]


class PicoDryRunProtocolTests(unittest.TestCase):
    def test_sequence_and_crc_round_trip(self):
        frame = encode_command(42, 7, 50, -200)
        self.assertEqual(decode_ready(encode_ready(42)), 42)
        self.assertEqual(decode_command(frame), (42, 7, 50, -200))
        self.assertEqual(decode_ack(encode_ack(42, 7, "ACTIVE")), (42, 7, "ACTIVE"))

    def test_host_and_pico_codecs_are_wire_compatible(self):
        path = ROOT / "simulation/dry_run_protocol.py"
        spec = importlib.util.spec_from_file_location("host_dry_run_protocol", path)
        host = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(host)
        command = host.encode_command(42, 9, 80, -300)
        self.assertEqual(host.decode_ready(encode_ready(42)), 42)
        self.assertEqual(decode_command(command), (42, 9, 80, -300))
        self.assertEqual(host.decode_ack(encode_ack(42, 9, "ACTIVE")), (42, 9, "ACTIVE"))

    def test_corrupt_frame_forces_stopped(self):
        state = DryRunReceiverState(42, timeout_ms=500)
        state.accept(encode_command(42, 1, 50, 0), 0)
        frame = bytearray(encode_command(42, 2, 60, 0))
        frame[-3] = ord("0") if frame[-3] != ord("0") else ord("1")
        with self.assertRaisesRegex(ProtocolError, "checksum"):
            state.accept(bytes(frame), 10)
        self.assertEqual(state.state, "STOPPED")
        self.assertEqual((state.linear_mm_s, state.angular_mrad_s), (0, 0))
        self.assertEqual(state.reason, "invalid_frame")

    def test_non_increasing_sequence_is_invalid(self):
        state = DryRunReceiverState(42)
        state.accept(encode_command(42, 3, 20, 0), 0)
        with self.assertRaisesRegex(ProtocolError, "not increasing"):
            state.accept(encode_command(42, 3, 20, 0), 1)
        self.assertEqual(state.state, "STOPPED")

    def test_receive_timeout_forces_stopped(self):
        state = DryRunReceiverState(42, timeout_ms=500)
        state.accept(encode_command(42, 1, 50, 100), 1000)
        self.assertFalse(state.poll(1500))
        self.assertTrue(state.poll(1501))
        self.assertEqual(state.state, "STOPPED")
        self.assertEqual(state.reason, "receive_timeout")

    def test_disconnect_forces_stopped(self):
        state = DryRunReceiverState(42)
        state.accept(encode_command(42, 1, 50, 0), 0)
        state.disconnect()
        self.assertEqual(state.state, "STOPPED")
        self.assertEqual(state.reason, "communication_lost")

    def test_explicit_stop_command(self):
        state = DryRunReceiverState(42)
        sequence, status = state.accept(encode_command(42, 1, 0, 0), 0)
        self.assertEqual((sequence, status), (1, "STOPPED"))
        self.assertEqual(state.reason, "stop_command")

    def test_out_of_range_value_is_rejected_and_stops(self):
        state = DryRunReceiverState(42)
        payload = b"CMD|2|42|1|501|0"
        frame = payload + b"|%04X\n" % crc16_ccitt(payload)
        with self.assertRaisesRegex(ProtocolError, "out of range"):
            state.accept(frame, 0)
        self.assertEqual(state.state, "STOPPED")

    def test_wrong_session_is_rejected_and_stops(self):
        state = DryRunReceiverState(42)
        with self.assertRaisesRegex(ProtocolError, "session mismatch"):
            state.accept(encode_command(41, 1, 50, 0), 0)
        self.assertEqual(state.state, "STOPPED")

    def test_pico_receiver_has_no_uart_or_roomba_oi_path(self):
        source = (ROOT / "pico/cmd_vel_dry_run_receiver.py").read_text()
        self.assertNotIn("import machine", source)
        self.assertNotIn("machine.UART", source)
        self.assertNotIn("uart.write", source.lower())
        self.assertNotIn("DRIVE_DIRECT", source)
        self.assertIn("UART=DISABLED", source)
        self.assertIn('"handshake_timeout"', source)
        self.assertIn("STOPPED_IDLE_TIMEOUT_MS = 2000", source)
        self.assertIn("connection.close()", source)

    def test_existing_drive_locks_remain_disabled(self):
        bridge = (ROOT / "simulation/roomba_bridge_tcp.py").read_text()
        relay = (ROOT / "pico/pico_relay.py").read_text()
        controller = (ROOT / "simulation/vision_controller.yaml").read_text()
        self.assertIn('declare_parameter("drive_enabled", False)', bridge)
        self.assertIn("ALLOW_NONZERO_DRIVE = False", relay)
        self.assertRegex(controller, r"(?m)^\s*control_enabled:\s*false$")

    def test_host_transport_sends_command_and_checks_ack_sequence(self):
        simulation = ROOT / "simulation"
        sys.path.insert(0, str(simulation))
        try:
            from pico_dry_run_transport import DryRunTcpTransport
        finally:
            sys.path.pop(0)

        class FakeSocket:
            def __init__(self):
                self.received = bytearray(encode_ready(42) + encode_ack(42, 1, "ACTIVE"))
                self.sent = []
                self.closed = False

            def settimeout(self, _timeout):
                pass

            def recv(self, size):
                if not self.received:
                    return b""
                result = bytes(self.received[:size])
                del self.received[:size]
                return result

            def sendall(self, data):
                self.sent.append(data)

            def close(self):
                self.closed = True

        connection = FakeSocket()
        transport = DryRunTcpTransport(
            "example", 9001, socket_factory=lambda *_args, **_kwargs: connection
        )
        transport.connect()
        self.assertEqual(transport.send(1, 50, 100), "ACTIVE")
        self.assertEqual(decode_command(connection.sent[0]), (42, 1, 50, 100))

    def test_host_transport_rejects_ack_from_an_old_session(self):
        simulation = ROOT / "simulation"
        sys.path.insert(0, str(simulation))
        try:
            from pico_dry_run_transport import DryRunTcpTransport
        finally:
            sys.path.pop(0)

        class FakeSocket:
            def __init__(self):
                self.received = bytearray(encode_ready(42) + encode_ack(41, 1, "ACTIVE"))

            def settimeout(self, _timeout):
                pass

            def recv(self, size):
                result = bytes(self.received[:size])
                del self.received[:size]
                return result

            def sendall(self, _data):
                pass

            def close(self):
                pass

        transport = DryRunTcpTransport(
            "example", 9001, socket_factory=lambda *_args, **_kwargs: FakeSocket()
        )
        transport.connect()
        with self.assertRaisesRegex(ConnectionError, "session or sequence"):
            transport.send(1, 50, 100)

    def test_reconnect_backoff_is_bounded_and_resets(self):
        simulation = ROOT / "simulation"
        sys.path.insert(0, str(simulation))
        try:
            from pico_dry_run_transport import ReconnectBackoff
        finally:
            sys.path.pop(0)

        backoff = ReconnectBackoff(initial_delay=1.0, max_delay=8.0)
        self.assertEqual([backoff.failed() for _ in range(5)], [1.0, 2.0, 4.0, 8.0, 8.0])
        backoff.reset()
        self.assertEqual(backoff.failed(), 1.0)

    def test_reconnect_gate_requires_fresh_zero_before_nonzero(self):
        simulation = ROOT / "simulation"
        sys.path.insert(0, str(simulation))
        try:
            from pico_dry_run_transport import ReconnectCommandGate
        finally:
            sys.path.pop(0)

        gate = ReconnectCommandGate()
        gate.connected()
        self.assertTrue(gate.allow(0.08, 0.0))
        gate.connected()
        self.assertFalse(gate.allow(0.08, 0.0))
        self.assertTrue(gate.allow(0.0, 0.0))
        self.assertTrue(gate.allow(0.08, 0.0))


if __name__ == "__main__":
    unittest.main()

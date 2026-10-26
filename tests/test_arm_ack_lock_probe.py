import socket
import struct
import unittest
from unittest import mock

from simulation import arm_ack_lock_probe as probe


class FakeSocket:
    def __init__(self, received, *, timeout_when_empty=False):
        self.received = bytearray(received)
        self.timeout_when_empty = timeout_when_empty
        self.sent = []
        self.closed = False

    def setsockopt(self, *_args):
        pass

    def settimeout(self, _timeout):
        pass

    def recv(self, size):
        if not self.received:
            if self.timeout_when_empty:
                raise socket.timeout("timed out")
            return b""
        chunk = bytes(self.received[:size])
        del self.received[:size]
        return chunk

    def sendall(self, payload):
        self.sent.append(bytes(payload))

    def close(self):
        self.closed = True


class ArmAckLockProbeTests(unittest.TestCase):
    def assert_non_driving_packets(self, connections):
        allowed_prefixes = (probe.STOP, probe.ARM)
        for connection in connections:
            for packet in connection.sent:
                self.assertTrue(
                    packet == probe.STOP or packet.startswith(allowed_prefixes[1]),
                    packet.hex(),
                )

    def test_locked_ack_and_reconnect_pass_without_drive(self):
        generation = 100
        ack = probe.ARM_RESPONSE_PREFIX + bytes([probe.ARM_REJECTED_LOCK])
        ack += struct.pack(">I", 0)
        first = FakeSocket(probe.READY + ack)
        second = FakeSocket(probe.READY)

        with mock.patch.object(
            probe.socket, "create_connection", side_effect=[first, second]
        ):
            result = probe.run_cycle(
                "192.0.2.1", 9000, 0.1, generation, cycle_id=1
            )

        self.assertTrue(result["pass"])
        self.assertEqual(result["highest_generation"], 0)
        self.assertTrue(result["reconnect_ready"])
        self.assertTrue(first.closed)
        self.assertTrue(second.closed)
        self.assert_non_driving_packets((first, second))

    def test_partial_ack_timeout_still_attempts_new_ready_session(self):
        generation = 101
        partial = probe.ARM_RESPONSE_PREFIX + bytes([probe.ARM_REJECTED_LOCK])
        first = FakeSocket(probe.READY + partial, timeout_when_empty=True)
        second = FakeSocket(probe.READY)

        with mock.patch.object(
            probe.socket, "create_connection", side_effect=[first, second]
        ):
            result = probe.run_cycle(
                "192.0.2.1", 9000, 0.1, generation, cycle_id=2
            )

        self.assertFalse(result["pass"])
        self.assertEqual(result["failure_reason"], "timeout")
        self.assertEqual(result["partial_bytes"], 3)
        self.assertTrue(result["reconnect_ready"])
        self.assert_non_driving_packets((first, second))


if __name__ == "__main__":
    unittest.main()

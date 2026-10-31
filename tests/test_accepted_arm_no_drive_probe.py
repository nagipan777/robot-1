import unittest

from simulation.accepted_arm_no_drive_probe import run_probe
from simulation.roomba_tcp_transport import RoombaDiagnosticError


class FakeTransport:
    def __init__(self):
        self.calls = []
        self.arm_calls = 0

    def connect(self):
        self.calls.append("connect")

    def arm(self, generation):
        self.calls.append(("arm", generation))
        self.arm_calls += 1
        if self.arm_calls == 2:
            raise RoombaDiagnosticError("ARM rejected: status=0x03")

    def send_stop(self):
        self.calls.append("stop")

    def verify_stop(self):
        self.calls.append("verify_stop")
        return 0x07, 0, 0

    def query_oi_mode(self):
        self.calls.append("oi_mode")
        return 2

    def close(self):
        self.calls.append("close")


class AcceptedArmNoDriveProbeTests(unittest.TestCase):
    def test_accepts_then_stops_and_rejects_stale_without_drive(self):
        transport = FakeTransport()

        result = run_probe(transport, 123)

        self.assertTrue(result["pass"])
        self.assertFalse(result["nonzero_drive_sent"])
        self.assertEqual(
            transport.calls,
            [
                "connect",
                ("arm", 123),
                "stop",
                "verify_stop",
                "oi_mode",
                ("arm", 123),
                "stop",
                "stop",
                "close",
            ],
        )

    def test_source_does_not_use_drive_api_or_drive_opcode(self):
        import inspect
        import simulation.accepted_arm_no_drive_probe as probe

        source = inspect.getsource(probe)
        self.assertNotIn("send_twist", source)
        self.assertNotIn("SEQUENCED_DRIVE", source)
        self.assertNotIn("DRIVE_DIRECT", source)


if __name__ == "__main__":
    unittest.main()

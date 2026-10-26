import unittest

from simulation.correlate_arm_logs import correlate, parse_pico_events


class CorrelateArmLogsTests(unittest.TestCase):
    def test_parse_and_classify_send_ok_but_host_timeout(self):
        pico = """\
TCP_ACCEPT at_ms=1 connection_id=4
ARM_RX at_ms=2 connection_id=4 generation=100
ACK_TX_BEGIN at_ms=3 connection_id=4 generation=100 planned=7
ACK_TX_OK at_ms=4 connection_id=4 generation=100 sent=7
TCP_CLOSE at_ms=5 connection_id=4 reason=peer_closed
"""
        host = {
            "results": [
                {
                    "generation": 100,
                    "ready": True,
                    "arm_rejected_lock": False,
                    "reconnect_ready": True,
                    "error": "ReceiveFailure: timeout after 0/7 bytes",
                }
            ]
        }

        result = correlate(pico, host)

        self.assertEqual(
            result["rows"][0]["classification"],
            "C_PICO_SEND_OK_HOST_DID_NOT_RECEIVE",
        )
        self.assertEqual(result["rows"][0]["pico_connection_ids"], [4])

    def test_send_error_and_missing_arm_are_distinguished(self):
        pico = """\
ARM_RX at_ms=2 connection_id=7 generation=200
ACK_TX_BEGIN at_ms=3 connection_id=7 generation=200 planned=7
ACK_TX_PARTIAL at_ms=4 connection_id=7 generation=200 sent=3
ACK_TX_ERROR at_ms=5 connection_id=7 generation=200 errno=104
"""
        host = {
            "results": [
                {"generation": 200},
                {"generation": 201, "ready": False},
            ]
        }

        result = correlate(pico, host)

        self.assertEqual(result["rows"][0]["classification"], "B_ACK_SEND_ERROR")
        self.assertEqual(result["rows"][1]["classification"], "A_NO_ARM_RX")

    def test_parser_ignores_unstructured_legacy_lines(self):
        events = parse_pico_events(
            "ARM accepted: generation=10\nARM_RX at_ms=2 connection_id=1 generation=10"
        )
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]["event"], "ARM_RX")


if __name__ == "__main__":
    unittest.main()

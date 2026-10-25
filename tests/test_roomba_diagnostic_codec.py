import unittest
from pathlib import Path

from pico.roomba_diagnostic_codec import (
    ENCODER_QUERY,
    decode_diagnostic_response,
    decode_encoder_response,
)


class RoombaDiagnosticCodecTests(unittest.TestCase):
    def test_decodes_charging_voltage_mode_and_wheel_drop(self):
        status = decode_diagnostic_response(
            bytes([2, 0x3A, 0x98, 0x05, 0xDC, 0x0B, 0xB8, 0x02, 2, 0x0C])
        )
        self.assertEqual(status["charging_state_name"], "full_charging")
        self.assertEqual(status["voltage_mv"], 15000)
        self.assertEqual(status["charge_mah"], 1500)
        self.assertEqual(status["capacity_mah"], 3000)
        self.assertEqual(status["charge_percent"], 50.0)
        self.assertTrue(status["charging_source_home_base"])
        self.assertEqual(status["oi_mode_name"], "safe")
        self.assertTrue(status["wheel_drop_left"])
        self.assertTrue(status["wheel_drop_right"])

    def test_does_not_assign_a_voltage_pass_fail_threshold(self):
        self.assertEqual(
            decode_diagnostic_response(bytes([0, 0, 1, 0, 0, 0, 0, 0, 1, 0]))["voltage_mv"],
            1,
        )

    def test_rejects_partial_and_invalid_enumerations(self):
        for response in (
            b"",
            b"\x00\x01",
            bytes([6, 0, 0, 0, 0, 0, 0, 0, 2, 0]),
            bytes([0, 0, 0, 0, 0, 0, 0, 0, 4, 0]),
            bytes([0, 0, 0, 0, 0, 0, 0, 4, 2, 0]),
            bytes([0, 0, 0, 0, 0, 0, 0, 0, 2, 0x10]),
        ):
            with self.subTest(response=response):
                with self.assertRaises(ValueError):
                    decode_diagnostic_response(response)

    def test_rejects_non_bytes(self):
        with self.assertRaises(TypeError):
            decode_diagnostic_response([0] * 10)

    def test_probe_contains_no_drive_opcode(self):
        source = (
            Path(__file__).resolve().parents[1] / "pico" / "roomba_uart_probe.py"
        ).read_text()
        self.assertNotIn("bytes([145", source)
        self.assertNotIn("DRIVE_DIRECT", source)
        self.assertIn("uart.write(DIAGNOSTIC_QUERY)", source)

    def test_encoder_query_and_decoder_are_drive_free(self):
        self.assertEqual(ENCODER_QUERY, bytes([149, 2, 43, 44]))
        self.assertNotIn(145, ENCODER_QUERY)
        self.assertEqual(decode_encoder_response(b"\xff\xfe\x00\x02"), (65534, 2))
        with self.assertRaises(ValueError):
            decode_encoder_response(b"\x00\x01")


if __name__ == "__main__":
    unittest.main()

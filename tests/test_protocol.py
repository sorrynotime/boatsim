import unittest

from boat_sim.protocol import (
    FrameParser,
    MOTOR_PAYLOAD,
    MSG_MOTOR_COMMAND,
    SENSOR_PAYLOAD,
    encode_frame,
    encode_sensor,
)


class ProtocolTests(unittest.TestCase):
    def test_parser_recovers_motor_frame_across_chunks(self):
        payload = MOTOR_PAYLOAD.pack(1234, -10000, 9999)
        frame = encode_frame(MSG_MOTOR_COMMAND, 42, payload)
        parser = FrameParser()
        self.assertEqual(parser.feed_motor_commands(b"bad" + frame[:5]), [])
        result = parser.feed_motor_commands(frame[5:])
        self.assertEqual(len(result), 1)
        self.assertEqual((result[0].left, result[0].right), (-10000, 9999))

    def test_bad_crc_is_rejected(self):
        frame = bytearray(encode_frame(MSG_MOTOR_COMMAND, 1, MOTOR_PAYLOAD.pack(1, 2, 3)))
        frame[-1] ^= 0xFF
        self.assertEqual(FrameParser().feed_motor_commands(frame), [])

    def test_sensor_uses_integer_e7_coordinates(self):
        frame = encode_sensor(7, 10, True, 0.5, -0.25, 30.1234567, 120.7654321, 1.5)
        payload = frame[8:-2]
        _, _, _, yaw_rate, latitude_e7, longitude_e7, _ = SENSOR_PAYLOAD.unpack(payload)
        self.assertAlmostEqual(yaw_rate, -0.25)
        self.assertEqual(latitude_e7, 301234567)
        self.assertEqual(longitude_e7, 1207654321)


if __name__ == "__main__":
    unittest.main()

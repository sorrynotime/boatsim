import struct
import unittest

from boat_sim.protocol import (
    MSG_NAV_CONFIG,
    MSG_NAV_FRAME,
    MSG_NAV_PLAN,
    MSG_NAV_RANGE,
    MSG_NAV_STATUS,
    MSG_NAV_TASK,
    FrameParser,
    decode_nav_status,
    decode_plan_segment,
    encode_nav_config,
    encode_nav_frame,
    encode_nav_task,
    encode_range,
)


class TestEncoders(unittest.TestCase):
    def test_frame_size(self):
        frame = encode_nav_frame(7, 12345, 1000, 30.0, 120.0)
        self.assertEqual(len(frame), 8 + 20 + 2)
        (message_type, sequence, payload), = FrameParser().feed(frame)
        self.assertEqual(message_type, MSG_NAV_FRAME)
        self.assertEqual(sequence, 7)
        self.assertEqual(struct.unpack("<IIiiHBB", payload),
                         (12345, 1000, 300_000_000, 1_200_000_000, 1, 1, 1))

    def test_range_roundtrip(self):
        frame = encode_range(3, 99, 4242, 3, 1, 2.5, 5.0, 5.0)
        (message_type, sequence, payload), = FrameParser().feed(frame)
        self.assertEqual(message_type, MSG_NAV_RANGE)
        self.assertEqual(sequence, 3)
        self.assertEqual(struct.unpack("<IIHBBfff", payload), (99, 4242, 1, 3, 1, 2.5, 5.0, 5.0))

    def test_task_size_and_fields(self):
        args = [3.0, 6.0, 3.0, 30.0, 1.5]
        frame = encode_nav_task(5, 77, 900, 6, 12, args)
        self.assertEqual(len(frame), 8 + 48 + 2)
        (message_type, _sequence, payload), = FrameParser().feed(frame)
        self.assertEqual(message_type, MSG_NAV_TASK)
        session, time_ms, frame_id, command, flags, request = struct.unpack_from("<IIHBBI", payload, 0)
        self.assertEqual((session, time_ms, frame_id, command, flags, request), (77, 900, 1, 6, 0, 12))
        values = struct.unpack_from("<8f", payload, 16)
        self.assertEqual(values[:5], (3.0, 6.0, 3.0, 30.0, 1.5))
        self.assertEqual(values[5:], (0.0, 0.0, 0.0))

    def test_config_size(self):
        frame = encode_nav_config(2, 5, 100, [5.0] * 12)
        self.assertEqual(len(frame), 8 + 60 + 2)
        (message_type, _sequence, payload), = FrameParser().feed(frame)
        self.assertEqual(message_type, MSG_NAV_CONFIG)


class TestStatusDecode(unittest.TestCase):
    def test_roundtrip(self):
        raw = struct.pack("<IIBBBB6H4I2f", 4242, 55, 0, 11, 1, 3, 180, 42, 10, 2, 3, 165,
                          9, 8, 1, 0, 0.5, 0.1)
        self.assertEqual(len(raw), 48)
        status = decode_nav_status(raw, 31)
        self.assertIsNotNone(status)
        assert status is not None
        self.assertEqual(status.sequence, 31)
        self.assertEqual(status.session, 4242)
        self.assertEqual(status.state, 11)
        self.assertEqual(status.state_name, "PLAN_WAIT")
        self.assertEqual(status.ack, 0)
        self.assertEqual(status.ack_name, "OK")
        self.assertEqual(status.total, 180)
        self.assertEqual(status.current, 42)

    def test_feed_messages_dispatch(self):
        raw = struct.pack("<IIBBBB6H4I2f", 1, 1, 0, 2, 1, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0, 0.0)
        frame = b"\xA5\x5A" + bytes([1, MSG_NAV_STATUS]) + struct.pack("<HH", 48, 9) + raw
        from boat_sim.protocol import crc16_ccitt
        frame += struct.pack("<H", crc16_ccitt(frame))
        messages = FrameParser().feed_messages(frame)
        self.assertEqual(len(messages), 1)
        self.assertEqual(messages[0].state, 2)
        self.assertEqual(messages[0].state_name, "SCAN_LAUNCH")


class TestPlanDecode(unittest.TestCase):
    def segment(self, count: int, start: int, total: int, flags: int, plan_id: int = 7) -> bytes:
        payload = struct.pack("<IHHHBB", 4242, plan_id, total, start, count, flags)
        for index in range(count):
            payload += struct.pack("<ff", 1.0 * (start + index), -2.0 * (start + index))
        return payload

    def frame_of(self, payload: bytes, sequence: int) -> bytes:
        body = b"\xA5\x5A" + bytes([1, MSG_NAV_PLAN]) + struct.pack("<HH", len(payload), sequence) + payload
        from boat_sim.protocol import crc16_ccitt
        return body + struct.pack("<H", crc16_ccitt(body))

    def test_decode_segment(self):
        payload = self.segment(3, 6, 13, 0)
        segment = decode_plan_segment(payload, 4)
        self.assertIsNotNone(segment)
        assert segment is not None
        self.assertEqual(segment.total, 13)
        self.assertEqual(segment.start_index, 6)
        self.assertEqual(segment.count, 3)
        self.assertEqual(segment.points[0], (6.0, -12.0))

    def test_bad_length_rejected(self):
        self.assertIsNone(decode_plan_segment(self.segment(3, 0, 13, 0)[:15], 1))

    def test_feed_dispatch(self):
        payload = self.segment(6, 0, 13, 1)
        messages = FrameParser().feed_messages(self.frame_of(payload, 21))
        self.assertEqual(len(messages), 1)
        segment = messages[0]
        self.assertEqual(segment.flags & 1, 1)
        self.assertEqual(len(segment.points), 6)


if __name__ == "__main__":
    unittest.main()

import unittest

from gait_core import JointBatch
from hardware.protocol import (
    PROTOCOL_VERSION,
    ProtocolError,
    crc32,
    decode_batch_frame,
    encode_batch,
)
from hardware.main import GoalReceiver


class ProtocolTest(unittest.TestCase):
    def make_batch(self, point_count=2):
        return JointBatch(
            goal_id=17,
            phase_name="test",
            points=tuple(tuple(float(index) for index in range(18)) for _ in range(point_count)),
            sample_period=0.02,
        )

    def test_crc32_matches_standard_vector(self):
        self.assertEqual(crc32(b"123456789"), 0xCBF43926)

    def test_batch_round_trip(self):
        frame = encode_batch(1234, self.make_batch())
        decoded = decode_batch_frame(frame)
        self.assertEqual(decoded["version"], PROTOCOL_VERSION)
        self.assertEqual(decoded["session_id"], 1234)
        self.assertEqual(decoded["goal_id"], 17)
        self.assertEqual(decoded["joint_count"], 18)
        self.assertEqual(decoded["point_count"], 2)
        self.assertEqual(decoded["sample_period_us"], 20_000)
        self.assertEqual(len(decoded["points"]), 2)

    def test_corruption_is_rejected(self):
        frame = bytearray(encode_batch(1234, self.make_batch()))
        frame[-5] ^= 0x80
        with self.assertRaisesRegex(ProtocolError, "BAD_CRC"):
            decode_batch_frame(bytes(frame))

    def test_oversized_batch_is_rejected(self):
        with self.assertRaisesRegex(ProtocolError, "BAD_POINT_COUNT"):
            encode_batch(1234, self.make_batch(65))

    def test_duplicate_goal_is_idempotent(self):
        decoded = decode_batch_frame(encode_batch(1234, self.make_batch()))
        receiver = GoalReceiver.__new__(GoalReceiver)
        receiver.last_session_id = None
        receiver.last_goal_id = None
        receiver.last_crc = None
        receiver.last_status = None
        replies = []
        playbacks = []
        receiver._reply = replies.append
        receiver._play_payload = lambda payload, count, period: playbacks.append(
            (payload, count, period)
        )

        receiver._handle_frame(decoded, decoded["payload"], decoded["crc"])
        receiver._handle_frame(decoded, decoded["payload"], decoded["crc"])

        self.assertEqual(len(playbacks), 1)
        self.assertEqual(replies[0], "ACK 1234 17")
        self.assertEqual(replies[1], "DONE 1234 17")
        self.assertEqual(replies[2], "DONE 1234 17")


if __name__ == "__main__":
    unittest.main()

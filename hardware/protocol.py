"""Versioned binary joint-batch protocol.

The implementation avoids CPython-only features so the same framing constants
and CRC implementation can be imported by Servo 2040 MicroPython.
"""

try:
    import ustruct as struct
except ImportError:  # CPython
    import struct

try:
    from binascii import crc32 as _native_crc32
except ImportError:  # Older MicroPython builds expose the module as ubinascii.
    from ubinascii import crc32 as _native_crc32


MAGIC = b"HXGB"
PROTOCOL_VERSION = 1
MESSAGE_BATCH = 1
JOINT_COUNT = 18
MAX_POINTS = 64
MIN_SAMPLE_PERIOD_US = 1_000
MAX_SAMPLE_PERIOD_US = 1_000_000

HEADER_FORMAT = "<4sBBHIIHHII"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
CRC_FORMAT = "<I"
CRC_SIZE = struct.calcsize(CRC_FORMAT)
POINT_FORMAT = "<18f"
POINT_SIZE = struct.calcsize(POINT_FORMAT)


class ProtocolError(Exception):
    def __init__(self, code, session_id=0, goal_id=0):
        super().__init__(code)
        self.code = code
        self.session_id = session_id
        self.goal_id = goal_id


def crc32(data):
    """Return an unsigned CRC-32 using the runtime's native implementation."""

    return _native_crc32(data) & 0xFFFFFFFF


def pack_header(session_id, goal_id, point_count, sample_period_us, payload_length):
    return struct.pack(
        HEADER_FORMAT,
        MAGIC,
        PROTOCOL_VERSION,
        MESSAGE_BATCH,
        HEADER_SIZE,
        session_id,
        goal_id,
        JOINT_COUNT,
        point_count,
        sample_period_us,
        payload_length,
    )


def unpack_header(header):
    if len(header) != HEADER_SIZE:
        raise ProtocolError("BAD_HEADER_SIZE")
    fields = struct.unpack(HEADER_FORMAT, header)
    return {
        "magic": fields[0],
        "version": fields[1],
        "message_type": fields[2],
        "header_size": fields[3],
        "session_id": fields[4],
        "goal_id": fields[5],
        "joint_count": fields[6],
        "point_count": fields[7],
        "sample_period_us": fields[8],
        "payload_length": fields[9],
    }


def validate_header(header):
    if header["magic"] != MAGIC:
        raise ProtocolError("BAD_MAGIC")
    if header["version"] != PROTOCOL_VERSION:
        raise ProtocolError("BAD_VERSION")
    if header["message_type"] != MESSAGE_BATCH:
        raise ProtocolError("BAD_MESSAGE_TYPE")
    if header["header_size"] != HEADER_SIZE:
        raise ProtocolError("BAD_HEADER_SIZE")
    if header["joint_count"] != JOINT_COUNT:
        raise ProtocolError("BAD_JOINT_COUNT")
    if not 1 <= header["point_count"] <= MAX_POINTS:
        raise ProtocolError("BAD_POINT_COUNT")
    if not MIN_SAMPLE_PERIOD_US <= header["sample_period_us"] <= MAX_SAMPLE_PERIOD_US:
        raise ProtocolError("BAD_SAMPLE_PERIOD")
    expected_payload = header["point_count"] * POINT_SIZE
    if header["payload_length"] != expected_payload:
        raise ProtocolError("BAD_PAYLOAD_SIZE")


def encode_batch(session_id, batch):
    point_count = len(batch.points)
    if not 1 <= point_count <= MAX_POINTS:
        raise ProtocolError("BAD_POINT_COUNT")
    sample_period_us = int(round(batch.sample_period * 1_000_000.0))
    payload_parts = []
    for point in batch.points:
        if len(point) != JOINT_COUNT:
            raise ProtocolError("BAD_JOINT_COUNT")
        payload_parts.append(struct.pack(POINT_FORMAT, *point))
    payload = b"".join(payload_parts)
    header = pack_header(
        session_id,
        batch.goal_id,
        point_count,
        sample_period_us,
        len(payload),
    )
    checksum = struct.pack(CRC_FORMAT, crc32(header + payload))
    return header + payload + checksum


def decode_batch_frame(frame):
    if len(frame) < HEADER_SIZE + CRC_SIZE:
        raise ProtocolError("TRUNCATED_FRAME")
    header_bytes = frame[:HEADER_SIZE]
    header = unpack_header(header_bytes)
    validate_header(header)
    expected_size = HEADER_SIZE + header["payload_length"] + CRC_SIZE
    if len(frame) != expected_size:
        raise ProtocolError("BAD_FRAME_SIZE")
    payload_end = HEADER_SIZE + header["payload_length"]
    payload = frame[HEADER_SIZE:payload_end]
    received_crc = struct.unpack(CRC_FORMAT, frame[payload_end:])[0]
    if received_crc != crc32(header_bytes + payload):
        raise ProtocolError("BAD_CRC")
    points = []
    for offset in range(0, len(payload), POINT_SIZE):
        points.append(struct.unpack(POINT_FORMAT, payload[offset : offset + POINT_SIZE]))
    header["payload"] = payload
    header["points"] = points
    header["crc"] = received_crc
    return header

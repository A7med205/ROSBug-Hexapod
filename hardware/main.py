#!/usr/bin/env python3
"""Servo 2040 firmware: validate and play discrete joint batches."""

import gc
import math
import sys
import time

try:
    import ustruct as struct
except ImportError:
    import struct

try:
    from servo import ServoCluster, servo2040
except ImportError:  # Allows host-side calibration tests without board modules.
    ServoCluster = None
    servo2040 = None

try:  # The board receives main.py and protocol.py in the same directory.
    from protocol import (
        CRC_FORMAT,
        CRC_SIZE,
        HEADER_FORMAT,
        HEADER_SIZE,
        JOINT_COUNT,
        MAGIC,
        POINT_FORMAT,
        POINT_SIZE,
        PROTOCOL_VERSION,
        ProtocolError,
        crc32,
        unpack_header,
        validate_header,
    )
except ImportError:  # Host-side import from the combined repository.
    from hardware.protocol import (
        CRC_FORMAT,
        CRC_SIZE,
        HEADER_FORMAT,
        HEADER_SIZE,
        JOINT_COUNT,
        MAGIC,
        POINT_FORMAT,
        POINT_SIZE,
        PROTOCOL_VERSION,
        ProtocolError,
        crc32,
        unpack_header,
        validate_header,
    )


SERVO_COUNT = 18
CENTER_PULSE = 1500.0
MIN_PULSE = 500.0
MAX_PULSE = 2500.0
JOINT_ORDER = (
    "j11", "j21", "j31", "j41", "j51", "j61",
    "j12", "j22", "j32", "j42", "j52", "j62",
    "j13", "j23", "j33", "j43", "j53", "j63",
)
LEG_JOINT_ORDER = (
    "j11", "j12", "j13",
    "j21", "j22", "j23",
    "j31", "j32", "j33",
    "j41", "j42", "j43",
    "j51", "j52", "j53",
    "j61", "j62", "j63",
)

# These calibrated values and conversions are intentionally unchanged from the
# original Servo 2040 firmware.
CALIBRATION = {
    "j11": (10.6667, 10.8889),
    "j12": (11.1556, 10.8889),
    "j13": (10.4444, 11.3333),
    "j21": (10.3333, 11.3333),
    "j22": (10.5556, 12.0000),
    "j23": (11.1667, 11.1667),
    "j31": (11.1111, 10.6667),
    "j32": (11.1111, 10.8889),
    "j33": (11.0000, 10.6667),
    "j41": (10.5556, 10.8889),
    "j42": (11.1111, 11.0000),
    "j43": (10.7778, 11.1111),
    "j51": (12.0000, 11.1111),
    "j52": (10.6667, 11.1111),
    "j53": (10.6667, 10.8889),
    "j61": (10.8889, 11.1111),
    "j62": (11.5556, 10.4444),
    "j63": (11.0000, 10.5556),
}
ROBOT_TO_SERVO = {
    "j11": (1.0, 0.0),
    "j12": (-1.0, -84.0),
    "j13": (1.0, -121.0),
    "j21": (1.0, 0.0),
    "j22": (-1.0, -69.0),
    "j23": (1.0, -114.0),
    "j31": (1.0, 0.0),
    "j32": (-1.0, -79.0),
    "j33": (1.0, -121.0),
    "j41": (1.0, 0.0),
    "j42": (-1.0, -74.0),
    "j43": (1.0, -118.0),
    "j51": (1.0, 0.0),
    "j52": (-1.0, -67.0),
    "j53": (1.0, -112.0),
    "j61": (1.0, 0.0),
    "j62": (-1.0, -78.0),
    "j63": (1.0, -121.0),
}


class GoalReceiver:
    def __init__(self):
        if ServoCluster is None or servo2040 is None:
            raise RuntimeError("Servo 2040 modules are unavailable")
        gc.collect()
        start_pin = servo2040.SERVO_1
        end_pin = getattr(servo2040, "SERVO_%d" % SERVO_COUNT)
        self.servos = ServoCluster(pio=0, sm=0, pins=list(range(start_pin, end_pin + 1)))
        self.servos.enable_all()
        self.joint_to_channel = {joint: index for index, joint in enumerate(JOINT_ORDER)}
        self.stream = getattr(sys.stdin, "buffer", sys.stdin)
        self.last_session_id = None
        self.last_goal_id = None
        self.last_crc = None
        self.last_status = None

    @staticmethod
    def _flush_stdout():
        flush = getattr(sys.stdout, "flush", None)
        if flush is not None:
            flush()

    def _reply(self, message):
        print(message)
        self._flush_stdout()

    @staticmethod
    def _clamp(value, minimum, maximum):
        return max(minimum, min(maximum, value))

    @staticmethod
    def _rad_to_deg(rad):
        return rad * (180.0 / math.pi)

    def robot_to_servo_deg(self, joint_name, robot_angle_rad):
        slope, intercept = ROBOT_TO_SERVO[joint_name]
        return slope * self._rad_to_deg(robot_angle_rad) + intercept

    def servo_deg_to_pulse(self, joint_name, servo_deg):
        slope_positive, slope_negative = CALIBRATION[joint_name]
        if servo_deg >= 0.0:
            return CENTER_PULSE + slope_positive * servo_deg
        return CENTER_PULSE + slope_negative * servo_deg

    def robot_angle_to_pulse(self, joint_name, robot_angle_rad):
        servo_deg = self.robot_to_servo_deg(joint_name, robot_angle_rad)
        pulse = self.servo_deg_to_pulse(joint_name, servo_deg)
        return self._clamp(pulse, MIN_PULSE, MAX_PULSE)

    def apply_goal(self, joint_values):
        pulses = [0.0] * SERVO_COUNT
        for index, joint_name in enumerate(LEG_JOINT_ORDER):
            channel = self.joint_to_channel[joint_name]
            pulses[channel] = self.robot_angle_to_pulse(joint_name, joint_values[index])
        for channel, pulse in enumerate(pulses):
            self.servos.pulse(channel, pulse)

    def _read_exact(self, count):
        chunks = []
        remaining = count
        while remaining:
            chunk = self.stream.read(remaining)
            if not chunk:
                continue
            if isinstance(chunk, str):
                chunk = chunk.encode("latin1")
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)

    def _read_magic(self):
        matched = 0
        while matched < len(MAGIC):
            value = self._read_exact(1)[0]
            if value == MAGIC[matched]:
                matched += 1
            else:
                matched = 1 if value == MAGIC[0] else 0
        return MAGIC

    @staticmethod
    def _is_finite(value):
        return value == value and value not in (float("inf"), -float("inf"))

    def _validate_payload_values(self, payload, point_count):
        for point_index in range(point_count):
            values = struct.unpack_from(POINT_FORMAT, payload, point_index * POINT_SIZE)
            for value in values:
                if not self._is_finite(value):
                    raise ProtocolError("NONFINITE_JOINT")

    def _wait_until(self, deadline):
        while True:
            remaining = time.ticks_diff(deadline, time.ticks_us())
            if remaining <= 0:
                return
            time.sleep_us(remaining)

    def _play_payload(self, payload, point_count, sample_period_us):
        start = time.ticks_us()
        for point_index in range(point_count):
            deadline = time.ticks_add(start, (point_index + 1) * sample_period_us)
            self._wait_until(deadline)
            values = struct.unpack_from(POINT_FORMAT, payload, point_index * POINT_SIZE)
            self.apply_goal(values)

    def _read_frame(self):
        magic = self._read_magic()
        header_bytes = magic + self._read_exact(HEADER_SIZE - len(MAGIC))
        header = unpack_header(header_bytes)
        try:
            validate_header(header)
            payload = self._read_exact(header["payload_length"])
            received_crc = struct.unpack(CRC_FORMAT, self._read_exact(CRC_SIZE))[0]
            computed_crc = crc32(header_bytes + payload)
            if received_crc != computed_crc:
                raise ProtocolError("BAD_CRC")
            self._validate_payload_values(payload, header["point_count"])
        except ProtocolError as exc:
            raise ProtocolError(
                exc.code,
                header["session_id"],
                header["goal_id"],
            )
        return header, payload, received_crc

    def _handle_frame(self, header, payload, received_crc):
        session_id = header["session_id"]
        goal_id = header["goal_id"]
        key_matches = session_id == self.last_session_id and goal_id == self.last_goal_id
        if key_matches:
            if received_crc != self.last_crc:
                self._reply("ERR %u %u ID_REUSE" % (session_id, goal_id))
            else:
                self._reply("%s %u %u" % (self.last_status, session_id, goal_id))
            return

        if session_id == self.last_session_id and self.last_goal_id is not None:
            if goal_id < self.last_goal_id:
                self._reply("ERR %u %u STALE_GOAL" % (session_id, goal_id))
                return

        self.last_session_id = session_id
        self.last_goal_id = goal_id
        self.last_crc = received_crc
        self.last_status = "ACK"
        gc.collect()
        self._reply("ACK %u %u" % (session_id, goal_id))
        try:
            gc.disable()
            self._play_payload(
                payload,
                header["point_count"],
                header["sample_period_us"],
            )
        except Exception as exc:
            self.last_status = "ERR"
            self._reply("ERR %u %u PLAYBACK_%r" % (session_id, goal_id, exc))
            return
        finally:
            gc.enable()
        self.last_status = "DONE"
        self._reply("DONE %u %u" % (session_id, goal_id))

    def run(self):
        self._reply("READY %u" % PROTOCOL_VERSION)
        while True:
            header = None
            try:
                header, payload, received_crc = self._read_frame()
                self._handle_frame(header, payload, received_crc)
            except ProtocolError as exc:
                self._reply(
                    "ERR %u %u %s"
                    % (exc.session_id, exc.goal_id, exc.code)
                )
            except Exception as exc:
                self._reply("ERR 0 0 %r" % (exc,))


def main():
    time.sleep(5.0)
    GoalReceiver().run()


if __name__ == "__main__":
    main()

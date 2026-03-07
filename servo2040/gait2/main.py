#!/usr/bin/env python3

import gc
import math
import sys
import uselect

from servo import ServoCluster, servo2040


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
CALIBRATION = {
    "j11": (10.6667, 10.8889),
    "j12": (10.4444, 11.7778),
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
    "j12": (-1.0, -78.0),
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
    def __init__(self) -> None:
        gc.collect()
        start_pin = servo2040.SERVO_1
        end_pin = getattr(servo2040, "SERVO_%d" % SERVO_COUNT)
        self.servos = ServoCluster(pio=0, sm=0, pins=list(range(start_pin, end_pin + 1)))
        self.servos.enable_all()
        self.joint_to_channel = {joint: idx for idx, joint in enumerate(JOINT_ORDER)}
        self.poll = uselect.poll()
        self.poll.register(sys.stdin, uselect.POLLIN)

    @staticmethod
    def _flush_stdout() -> None:
        flush = getattr(sys.stdout, "flush", None)
        if flush is not None:
            flush()

    def _reply(self, message: str) -> None:
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

    def apply_goal(self, joint_values) -> None:
        pulses = [0.0] * SERVO_COUNT
        for idx, joint_name in enumerate(LEG_JOINT_ORDER):
            channel = self.joint_to_channel[joint_name]
            pulses[channel] = self.robot_angle_to_pulse(joint_name, joint_values[idx])
        for channel, pulse in enumerate(pulses):
            self.servos.pulse(channel, pulse)

    def handle_line(self, line: str) -> None:
        parts = line.strip().split()
        if not parts:
            return
        command = parts[0].upper()

        try:
            if command == "PING":
                self._reply("PONG")
                return
            if command == "GOAL":
                if len(parts) != len(LEG_JOINT_ORDER) + 1:
                    self._reply("ERR BAD_GOAL_COUNT")
                    return
                joint_values = [float(value) for value in parts[1:]]
                self.apply_goal(joint_values)
                self._reply("OK GOAL")
                return
            self._reply("ERR UNKNOWN_CMD")
        except Exception as exc:
            self._reply("ERR %r" % (exc,))

    def run(self) -> None:
        self._reply("READY")
        while True:
            if self.poll.poll(0):
                line = sys.stdin.readline()
                if line:
                    self.handle_line(line)


def main() -> None:
    GoalReceiver().run()


if __name__ == "__main__":
    main()

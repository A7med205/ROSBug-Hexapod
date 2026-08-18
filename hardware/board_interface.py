#!/usr/bin/env python3
"""Keyboard-to-Servo-2040 host adapter for the shared gait core."""

from __future__ import annotations

import argparse
import secrets
import sys
import time
from pathlib import Path
from typing import Callable

try:
    from gait_core import CommandKind, JointBatch, LiteGaitCoordinator
    from common.keyboard_input import KeyboardInput, help_text
except ImportError:
    # Support direct execution as well as ``python -m hardware.board_interface``.
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from gait_core import CommandKind, JointBatch, LiteGaitCoordinator
    from common.keyboard_input import KeyboardInput, help_text

import serial

try:
    from .protocol import PROTOCOL_VERSION, encode_batch
except ImportError:
    from protocol import PROTOCOL_VERSION, encode_batch


class BoardBatchExecutor:
    def __init__(self, port: str, baudrate: int, response_timeout: float) -> None:
        self.serial = serial.Serial(port, baudrate, timeout=0.02)
        self.response_timeout = response_timeout
        self.session_id = secrets.randbits(32)

    def close(self) -> None:
        try:
            self.serial.close()
        except Exception:
            pass

    def _read_response(self):
        raw = self.serial.readline()
        if not raw:
            return None
        line = raw.decode("utf-8", errors="replace").strip()
        if not line:
            return None
        fields = line.split(maxsplit=3)
        if fields[0] == "READY":
            if len(fields) > 1 and int(fields[1]) != PROTOCOL_VERSION:
                raise RuntimeError(
                    f"board protocol {fields[1]} does not match host {PROTOCOL_VERSION}"
                )
            print(f"board: {line}")
            return None
        if fields[0] not in ("ACK", "DONE", "ERR") or len(fields) < 3:
            print(f"board: {line}")
            return None
        try:
            session_id = int(fields[1])
            goal_id = int(fields[2])
        except ValueError:
            print(f"board: {line}")
            return None
        return fields[0], session_id, goal_id, fields[3] if len(fields) > 3 else ""

    def execute(self, batch: JointBatch, poll_input: Callable[[], None]) -> bool:
        frame = encode_batch(self.session_id, batch)
        self.serial.write(frame)
        self.serial.flush()

        acknowledged = False
        ack_deadline = time.monotonic() + self.response_timeout
        done_deadline = ack_deadline + batch.point_count * batch.sample_period + self.response_timeout
        retransmitted = False

        while time.monotonic() < done_deadline:
            poll_input()
            response = self._read_response()
            if response is not None:
                status, session_id, goal_id, detail = response
                if session_id != self.session_id or goal_id != batch.goal_id:
                    # A delayed idempotent response from an earlier retry.
                    continue
                if status == "ERR":
                    print(f"board rejected goal {goal_id}: {detail}", file=sys.stderr)
                    return False
                if status == "ACK":
                    acknowledged = True
                    done_deadline = (
                        time.monotonic()
                        + batch.point_count * batch.sample_period
                        + self.response_timeout
                    )
                elif status == "DONE":
                    return self._hold_after(batch, poll_input)

            if not acknowledged and time.monotonic() >= ack_deadline:
                if retransmitted:
                    print(f"goal {batch.goal_id}: board acknowledgment timeout", file=sys.stderr)
                    return False
                # The same session/goal pair is an idempotent retry.  Firmware
                # must acknowledge its current status without replaying it.
                self.serial.write(frame)
                self.serial.flush()
                retransmitted = True
                ack_deadline = time.monotonic() + self.response_timeout
                done_deadline = (
                    ack_deadline
                    + batch.point_count * batch.sample_period
                    + self.response_timeout
                )

        print(f"goal {batch.goal_id}: board completion timeout", file=sys.stderr)
        return False

    @staticmethod
    def _hold_after(batch: JointBatch, poll_input: Callable[[], None]) -> bool:
        deadline = time.monotonic() + batch.hold_after
        while time.monotonic() < deadline:
            poll_input()
            time.sleep(min(0.02, max(0.0, deadline - time.monotonic())))
        return True


def _describe_command(command) -> str:
    if command.kind == CommandKind.WALK:
        return f"trajectory {command.trajectory_id}"
    return command.kind


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    # The firmware intentionally waits five seconds after board boot before it
    # starts its receiver, so the first acknowledgment needs a longer window.
    parser.add_argument("--response-timeout", type=float, default=8.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    print("Building shared gait templates...")
    coordinator = LiteGaitCoordinator()
    executor = BoardBatchExecutor(args.port, args.baudrate, args.response_timeout)
    quit_requested = False
    exit_code = 0

    try:
        with KeyboardInput() as keyboard:
            print(help_text("Hexapod hardware controller"))

            def poll_input() -> None:
                nonlocal quit_requested
                result = keyboard.poll(0.0)
                for notice in result.notices:
                    print(notice)
                if result.command is not None:
                    accepted = coordinator.request(result.command)
                    description = _describe_command(result.command)
                    print(f"command {'accepted' if accepted else 'ignored'}: {description}")
                quit_requested = quit_requested or result.quit_requested

            while not quit_requested:
                poll_input()
                batch = coordinator.next_batch()
                if batch is None:
                    time.sleep(0.02)
                    continue
                print(
                    f"goal {batch.goal_id}: {batch.phase_name}, "
                    f"{batch.point_count} points"
                )
                succeeded = executor.execute(batch, poll_input)
                coordinator.complete_batch(batch.goal_id, succeeded)
                if not succeeded:
                    exit_code = 1
                    break
    except KeyboardInterrupt:
        exit_code = 130
    finally:
        executor.close()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()

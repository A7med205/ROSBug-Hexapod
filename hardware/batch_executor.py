"""Servo 2040 batch executor independent of keyboard input."""

from __future__ import annotations

import secrets
import time
from typing import Callable

import serial

from robot_core import (
    BatchExecutionResult,
    BatchExecutionStatus,
    JointBatch,
)

from .protocol import PROTOCOL_VERSION, encode_batch


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
                    f"board protocol {fields[1]} does not match host "
                    f"{PROTOCOL_VERSION}"
                )
            return None
        if fields[0] not in ("ACK", "DONE", "ERR") or len(fields) < 3:
            return None
        try:
            session_id = int(fields[1])
            goal_id = int(fields[2])
        except ValueError:
            return None
        return fields[0], session_id, goal_id, fields[3] if len(fields) > 3 else ""

    def execute(
        self,
        batch: JointBatch,
        poll_commands: Callable[[], None],
    ) -> BatchExecutionResult:
        try:
            frame = encode_batch(self.session_id, batch)
            self.serial.write(frame)
            self.serial.flush()

            acknowledged = False
            ack_deadline = time.monotonic() + self.response_timeout
            done_deadline = (
                ack_deadline
                + batch.point_count * batch.sample_period
                + self.response_timeout
            )
            retransmitted = False

            while time.monotonic() < done_deadline:
                poll_commands()
                response = self._read_response()
                if response is not None:
                    status, session_id, goal_id, detail = response
                    if session_id != self.session_id or goal_id != batch.goal_id:
                        continue
                    if status == "ERR":
                        return BatchExecutionResult(
                            batch.goal_id,
                            BatchExecutionStatus.REJECTED,
                            detail or "board rejected batch",
                        )
                    if status == "ACK":
                        acknowledged = True
                        done_deadline = (
                            time.monotonic()
                            + batch.point_count * batch.sample_period
                            + self.response_timeout
                        )
                    elif status == "DONE":
                        return self._hold_after(batch, poll_commands)

                if not acknowledged and time.monotonic() >= ack_deadline:
                    if retransmitted:
                        return BatchExecutionResult(
                            batch.goal_id,
                            BatchExecutionStatus.ACK_TIMEOUT,
                            "board acknowledgment timeout",
                        )
                    self.serial.write(frame)
                    self.serial.flush()
                    retransmitted = True
                    ack_deadline = time.monotonic() + self.response_timeout
                    done_deadline = (
                        ack_deadline
                        + batch.point_count * batch.sample_period
                        + self.response_timeout
                    )

            return BatchExecutionResult(
                batch.goal_id,
                BatchExecutionStatus.COMPLETION_TIMEOUT,
                "board completion timeout",
            )
        except Exception as error:
            return BatchExecutionResult(
                batch.goal_id,
                BatchExecutionStatus.TRANSPORT_ERROR,
                str(error),
            )

    @staticmethod
    def _hold_after(
        batch: JointBatch,
        poll_commands: Callable[[], None],
    ) -> BatchExecutionResult:
        deadline = time.monotonic() + batch.hold_after
        while time.monotonic() < deadline:
            poll_commands()
            time.sleep(min(0.02, max(0.0, deadline - time.monotonic())))
        return BatchExecutionResult(
            batch.goal_id,
            BatchExecutionStatus.COMPLETED,
            "board batch completed",
        )

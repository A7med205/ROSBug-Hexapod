"""Structured results shared by hardware and simulation batch executors."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Protocol

from .motion_batch import JointBatch


class BatchExecutionStatus:
    COMPLETED = "completed"
    REJECTED = "rejected"
    ACK_TIMEOUT = "ack_timeout"
    COMPLETION_TIMEOUT = "completion_timeout"
    TRANSPORT_ERROR = "transport_error"
    SHUTDOWN = "shutdown"


@dataclass(frozen=True)
class BatchExecutionResult:
    goal_id: int
    status: str
    detail: str = ""

    @property
    def succeeded(self) -> bool:
        return self.status == BatchExecutionStatus.COMPLETED


class BatchExecutor(Protocol):
    """Transport-neutral interface implemented by serial and ROS executors."""

    def execute(
        self,
        batch: JointBatch,
        poll_commands: Callable[[], None],
    ) -> BatchExecutionResult: ...

    def close(self) -> None: ...

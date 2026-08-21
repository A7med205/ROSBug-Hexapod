"""Structured command and coordinator feedback for external frontends."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


class CoordinatorEventKind:
    COMMAND_ACCEPTED = "command_accepted"
    COMMAND_REJECTED = "command_rejected"
    OPERATION_REPLACED = "operation_replaced"
    BATCH_COMPLETED = "batch_completed"
    OPERATION_COMPLETED = "operation_completed"
    OPERATION_FAILED = "operation_failed"


@dataclass(frozen=True)
class CommandFeedback:
    accepted: bool
    code: str
    detail: str
    operation_id: Optional[int] = None


@dataclass(frozen=True)
class CoordinatorEvent:
    kind: str
    operation_id: Optional[int]
    command_kind: Optional[str]
    state: str
    mode: str
    detail: str = ""


@dataclass(frozen=True)
class CoordinatorStatus:
    state: str
    mode: str
    requested_mode: str
    operation_id: Optional[int]
    command_kind: Optional[str]
    pending_goal_id: Optional[int]
    auto_requested_steps: Optional[int]
    auto_completed_half_steps: Optional[int]
    auto_remaining_half_steps: Optional[int]

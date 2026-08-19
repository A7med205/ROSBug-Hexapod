"""Transport-neutral joint trajectory batches shared by every motion core."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple


JointPoint = Tuple[float, ...]


@dataclass(frozen=True)
class JointBatch:
    goal_id: int
    phase_name: str
    points: Tuple[JointPoint, ...]
    sample_period: float
    hold_after: float = 0.0
    trajectory_id: int = 0

    @property
    def point_count(self) -> int:
        return len(self.points)


class GoalIdAllocator:
    """Allocate monotonically increasing IDs across gait and posture batches."""

    def __init__(self, first_goal_id: int = 1) -> None:
        if first_goal_id < 0:
            raise ValueError("first_goal_id must be non-negative")
        self._next_goal_id = first_goal_id

    def allocate(self) -> int:
        goal_id = self._next_goal_id
        self._next_goal_id += 1
        return goal_id

"""Transport-independent robot definitions shared by motion generators."""

from .control import Command, CommandKind, ControllerMode, PostureAxis
from .execution import (
    BatchExecutionResult,
    BatchExecutionStatus,
    BatchExecutor,
)
from .feedback import (
    CommandFeedback,
    CoordinatorEvent,
    CoordinatorEventKind,
    CoordinatorStatus,
)
from .model import (
    BasePose3D,
    FramePose,
    HexapodModel,
    JointAngles,
    LegInfo,
    Point3D,
    RobotConfig,
    UnreachableTipError,
)
from .motion_batch import GoalIdAllocator, JointBatch, JointPoint

__all__ = [
    "BasePose3D",
    "BatchExecutionResult",
    "BatchExecutionStatus",
    "BatchExecutor",
    "Command",
    "CommandFeedback",
    "CommandKind",
    "ControllerMode",
    "CoordinatorEvent",
    "CoordinatorEventKind",
    "CoordinatorStatus",
    "FramePose",
    "GoalIdAllocator",
    "HexapodModel",
    "JointAngles",
    "JointBatch",
    "JointPoint",
    "LegInfo",
    "Point3D",
    "PostureAxis",
    "RobotConfig",
    "UnreachableTipError",
]

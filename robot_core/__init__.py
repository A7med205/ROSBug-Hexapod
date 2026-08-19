"""Transport-independent robot definitions shared by motion generators."""

from .control import Command, CommandKind, ControllerMode, PostureAxis
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
    "Command",
    "CommandKind",
    "ControllerMode",
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

"""Commands and operating modes understood by the shared motion coordinator."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


class CommandKind:
    STARTUP = "startup"
    SKIP_STARTUP = "skip_startup"
    SIT_DOWN = "sit_down"
    STOP = "stop"
    TOGGLE_MODE = "toggle_mode"
    SET_MODE = "set_mode"
    WALK = "walk"
    POSTURE = "posture"
    RESET_TILT = "reset_tilt"


class ControllerMode:
    NORMAL = "normal"
    AUTO = "auto"
    POSTURE = "posture"

    ORDER = (NORMAL, AUTO, POSTURE)

    @classmethod
    def next(cls, mode: str) -> str:
        try:
            index = cls.ORDER.index(mode)
        except ValueError as exc:
            raise ValueError(f"unknown controller mode: {mode}") from exc
        return cls.ORDER[(index + 1) % len(cls.ORDER)]


class PostureAxis:
    ELEVATION = "elevation"
    PITCH = "pitch"
    ROLL = "roll"

    ALL = (ELEVATION, PITCH, ROLL)


@dataclass(frozen=True)
class Command:
    kind: str
    trajectory_id: int = 0
    steps: Optional[int] = None
    mode: Optional[str] = None
    posture_axis: Optional[str] = None
    posture_delta: Optional[float] = None

    @classmethod
    def startup(cls) -> "Command":
        return cls(CommandKind.STARTUP)

    @classmethod
    def skip_startup(cls) -> "Command":
        return cls(CommandKind.SKIP_STARTUP)

    @classmethod
    def sit_down(cls) -> "Command":
        return cls(CommandKind.SIT_DOWN)

    @classmethod
    def stop(cls) -> "Command":
        return cls(CommandKind.STOP)

    @classmethod
    def toggle_mode(cls) -> "Command":
        return cls(CommandKind.TOGGLE_MODE)

    @classmethod
    def set_mode(cls, mode: str) -> "Command":
        return cls(CommandKind.SET_MODE, mode=mode)

    @classmethod
    def walk(cls, trajectory_id: int, steps: Optional[int] = None) -> "Command":
        return cls(CommandKind.WALK, trajectory_id=trajectory_id, steps=steps)

    @classmethod
    def posture(cls, axis: str, delta: float) -> "Command":
        return cls(
            CommandKind.POSTURE,
            posture_axis=axis,
            posture_delta=delta,
        )

    @classmethod
    def reset_tilt(cls) -> "Command":
        return cls(CommandKind.RESET_TILT)

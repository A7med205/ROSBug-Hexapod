"""Nonblocking terminal input shared by the hardware and ROS adapters."""

from __future__ import annotations

import select
import sys
import termios
import tty
from dataclasses import dataclass
from typing import List, Optional

from gait_core import Command


LINE_KEYS = {"w": 1, "d": 2, "s": 8, "a": 9}
DIAGONAL_KEYS = {"q": 4, "e": 3, "z": 10, "c": 11}
ORBIT_KEYS = {"q": 6, "e": 12, "z": 13, "c": 5}
ROTATION_KEYS = {"o": 14, "p": 7}


@dataclass(frozen=True)
class KeyboardPoll:
    command: Optional[Command]
    quit_requested: bool
    notices: List[str]


class KeyboardInput:
    def __init__(self) -> None:
        self.mode = "diagonal"
        self.step_prefix = ""
        self._old_settings = None

    def __enter__(self) -> "KeyboardInput":
        if not sys.stdin.isatty():
            raise RuntimeError("keyboard input requires an interactive terminal")
        self._old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        return self

    def __exit__(self, _exc_type, _exc, _traceback) -> None:
        if self._old_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)
            self._old_settings = None

    def _movement_trajectory(self, key: str) -> Optional[int]:
        if key in ROTATION_KEYS:
            return ROTATION_KEYS[key]
        if key in LINE_KEYS:
            return LINE_KEYS[key]
        table = DIAGONAL_KEYS if self.mode == "diagonal" else ORBIT_KEYS
        return table.get(key)

    def feed_key(self, key: str) -> KeyboardPoll:
        key = key.lower()
        notices: List[str] = []

        if key == "x":
            return KeyboardPoll(None, True, notices)
        if key in ("\x08", "\x7f"):
            self.step_prefix = self.step_prefix[:-1]
            notices.append(f"step count: {self.step_prefix or '(empty)'}")
            return KeyboardPoll(None, False, notices)
        if key == "\x1b":
            self.step_prefix = ""
            notices.append("step count cleared")
            return KeyboardPoll(None, False, notices)
        if key.isdigit():
            if key == "0" and not self.step_prefix:
                return KeyboardPoll(Command.stop(), False, notices)
            self.step_prefix += key
            notices.append(f"step count: {self.step_prefix}")
            return KeyboardPoll(None, False, notices)

        if key == "m":
            self.step_prefix = ""
            self.mode = "orbit" if self.mode == "diagonal" else "diagonal"
            notices.append(f"q/e/z/c mode: {self.mode}")
            return KeyboardPoll(None, False, notices)
        if key == "t":
            self.step_prefix = ""
            return KeyboardPoll(Command.toggle_mode(), False, notices)
        if key == "u":
            self.step_prefix = ""
            return KeyboardPoll(Command.startup(), False, notices)
        if key == "k":
            self.step_prefix = ""
            return KeyboardPoll(Command.skip_startup(), False, notices)

        trajectory_id = self._movement_trajectory(key)
        if trajectory_id is None:
            return KeyboardPoll(None, False, notices)
        steps = int(self.step_prefix) if self.step_prefix else None
        self.step_prefix = ""
        return KeyboardPoll(Command.walk(trajectory_id, steps), False, notices)

    def poll(self, timeout: float = 0.0) -> KeyboardPoll:
        command: Optional[Command] = None
        quit_requested = False
        notices: List[str] = []

        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        while ready:
            key = sys.stdin.read(1)
            if not key:
                break
            mapped = self.feed_key(key)
            quit_requested = quit_requested or mapped.quit_requested
            notices.extend(mapped.notices)
            if mapped.command is not None:
                # Commands are deliberately latched, not queued. Draining the
                # terminal leaves only the most recently entered one.
                command = mapped.command
            ready, _, _ = select.select([sys.stdin], [], [], 0.0)

        return KeyboardPoll(command, quit_requested, notices)


def help_text(controller_name: str) -> str:
    return "\n".join(
        (
            controller_name,
            "Startup/stand: u",
            "Skip startup (assert already standing): k",
            "Graceful stop: 0",
            "Toggle normal/auto mode: t",
            "Auto counted motion: type count then movement (for example 5w)",
            "Lines: w(+Y), d(+X), s(-Y), a(-X)",
            "Self rotation: o(CCW), p(CW)",
            "Toggle q/e/z/c diagonal/orbit mode: m",
            "Quit after the current batch: x",
        )
    )

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
        if not sys.stdin.isatty():
            raise RuntimeError("keyboard input requires an interactive terminal")
        self.mode = "diagonal"
        self._old_settings = None

    def __enter__(self) -> "KeyboardInput":
        self._old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        return self

    def __exit__(self, _exc_type, _exc, _traceback) -> None:
        if self._old_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)
            self._old_settings = None

    def _map_key(self, key: str) -> Optional[Command]:
        if key == "u":
            return Command.startup()
        if key == "0":
            return Command.stop()
        if key in ROTATION_KEYS:
            return Command.walk(ROTATION_KEYS[key])
        if key in LINE_KEYS:
            return Command.walk(LINE_KEYS[key])
        table = DIAGONAL_KEYS if self.mode == "diagonal" else ORBIT_KEYS
        trajectory_id = table.get(key)
        return Command.walk(trajectory_id) if trajectory_id is not None else None

    def poll(self, timeout: float = 0.0) -> KeyboardPoll:
        command: Optional[Command] = None
        quit_requested = False
        notices: List[str] = []

        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        while ready:
            key = sys.stdin.read(1)
            if not key:
                break
            key = key.lower()
            if key == "x":
                quit_requested = True
            elif key == "m":
                self.mode = "orbit" if self.mode == "diagonal" else "diagonal"
                notices.append(f"q/e/z/c mode: {self.mode}")
            else:
                mapped = self._map_key(key)
                if mapped is not None:
                    # Commands are deliberately latched, not queued.  Draining
                    # the terminal leaves only the most recently entered one.
                    command = mapped
            ready, _, _ = select.select([sys.stdin], [], [], 0.0)

        return KeyboardPoll(command, quit_requested, notices)


def help_text(controller_name: str) -> str:
    return "\n".join(
        (
            controller_name,
            "Startup/stand: u",
            "Graceful stop: 0",
            "Lines: w(+Y), d(+X), s(-Y), a(-X)",
            "Self rotation: o(CCW), p(CW)",
            "Toggle q/e/z/c diagonal/orbit mode: m",
            "Quit after the current batch: x",
        )
    )

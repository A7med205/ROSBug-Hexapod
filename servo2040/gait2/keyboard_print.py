#!/usr/bin/env python3

import sys
import termios
import tty
from typing import List

from keyboard import LiteController, print_help


class PrintBoardClient:
    def close(self) -> None:
        return

    def wait_for_ping(self, total_timeout_sec: float = 8.0, retry_interval_sec: float = 0.25) -> bool:
        print(">> PING")
        print("<< PONG")
        return True

    def send_goal(self, joint_values: List[float]) -> bool:
        return True


class PrintLiteController(LiteController):
    def __init__(self) -> None:
        self.mode = "diagonal"
        self.board = PrintBoardClient()
        self._init_state()
        self.running = True

    def _execute_standard_phase(
        self,
        phase_name: str,
        trajectory_type_id: int,
        tripod_a_mode,
        tripod_b_mode,
    ) -> bool:
        print(phase_name)
        return super()._execute_standard_phase(
            phase_name=phase_name,
            trajectory_type_id=trajectory_type_id,
            tripod_a_mode=tripod_a_mode,
            tripod_b_mode=tripod_b_mode,
        )


def main() -> None:
    controller = PrintLiteController()
    old_settings = termios.tcgetattr(sys.stdin)
    print_help()
    exit_code = 0
    try:
        tty.setraw(sys.stdin.fileno())
        exit_code = controller.run()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        controller.board.close()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()

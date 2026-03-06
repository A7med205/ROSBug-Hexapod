#!/usr/bin/env python3

import select
import sys
import termios
import time
import tty

import serial


LINE_KEYS = {
    "w": 1,   # +Y
    "d": 2,   # +X
    "s": 8,   # -Y
    "a": 9,   # -X
}
DIAGONAL_KEYS = {
    "q": 4,   # +Y,-X
    "e": 3,   # +Y,+X
    "z": 10,  # -Y,-X
    "c": 11,  # -Y,+X
}
ORBIT_KEYS = {
    "q": 6,   # center -X
    "e": 12,  # reverse of center +X
    "z": 13,  # reverse of center -X
    "c": 5,   # center +X
}
ROTATION_KEYS = {
    "o": 14,  # self CCW
    "p": 7,   # self CW
}


class LiteControllerKeyboardPublisher:
    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 115200) -> None:
        self.port = port
        self.baudrate = baudrate
        self.mode = "diagonal"
        self.serial = serial.Serial(self.port, self.baudrate, timeout=0.1)

    def close(self) -> None:
        try:
            self.serial.close()
        except Exception:
            pass

    def log(self, message: str) -> None:
        print(message)

    def _read_response(self, expected_prefixes: tuple[str, ...], timeout_sec: float) -> str:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            line = self.serial.readline().decode(errors="ignore").strip()
            if not line:
                continue
            self.log(f"<< {line}")
            if line in ("PING",) or line.startswith("TRAJ "):
                continue
            if any(line.startswith(prefix) for prefix in expected_prefixes):
                return line
        return ""

    def wait_for_ready(self, timeout_sec: float = 5.0) -> None:
        self.log(f"waiting for board on {self.port}")
        self.serial.reset_input_buffer()
        ready = self._read_response(("READY",), timeout_sec)
        if ready == "READY":
            return
        self.log("didn't receive READY, continuing anyway")

    def send_line(self, line: str, expected_prefixes: tuple[str, ...], timeout_sec: float = 2.0) -> str:
        self.serial.write((line + "\n").encode("utf-8"))
        self.serial.flush()
        return self._read_response(expected_prefixes, timeout_sec)

    def publish_trajectory(self, value: int) -> None:
        self.log(f">> TRAJ {value}")
        response = self.send_line(f"TRAJ {value}", ("OK TRAJ", "ERR"), 5.0)
        if not response:
            self.log("no trajectory acknowledgement from board")

    def ping(self) -> None:
        self.log(">> PING")
        response = self.send_line("PING", ("PONG", "ERR"), 5.0)
        if not response:
            self.log("no ping response from board")

    def toggle_mode(self) -> None:
        self.mode = "orbit" if self.mode == "diagonal" else "diagonal"
        self.log(f"q/e/z/c mode: {self.mode}")

    def map_key(self, key: str) -> int | None:
        k = key.lower()
        if k == "0":
            return 0
        if k in ROTATION_KEYS:
            return ROTATION_KEYS[k]
        if k in LINE_KEYS:
            return LINE_KEYS[k]
        table = DIAGONAL_KEYS if self.mode == "diagonal" else ORBIT_KEYS
        return table.get(k)


def read_key(timeout_sec: float = 0.05) -> str:
    ready, _, _ = select.select([sys.stdin], [], [], timeout_sec)
    return sys.stdin.read(1) if ready else ""


def print_help() -> None:
    print("Lite controller keyboard serial publisher")
    print("Stop: 0")
    print("Lines: w(+Y), d(+X), s(-Y), a(-X)")
    print("Self rotation: o(CCW), p(CW)")
    print("q/e/z/c mode toggle: press m (diagonal <-> orbit)")
    print("Diagonal mode q/e/z/c: (+Y,-X), (+Y,+X), (-Y,-X), (-Y,+X)")
    print("Orbit mode q/e/z/c: (-X center), reverse(+X), reverse(-X), (+X center)")
    print("Quit: x")


def main() -> None:
    node = LiteControllerKeyboardPublisher()
    old_settings = termios.tcgetattr(sys.stdin)
    print_help()

    try:
        node.wait_for_ready()
        node.ping()
        tty.setraw(sys.stdin.fileno())
        running = True
        while running:
            key = read_key(0.03)
            if not key:
                continue
            k = key.lower()
            if k == "x":
                running = False
                continue
            if k == "m":
                node.toggle_mode()
                continue
            value = node.map_key(key)
            if value is not None:
                node.publish_trajectory(value)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.close()


if __name__ == "__main__":
    main()

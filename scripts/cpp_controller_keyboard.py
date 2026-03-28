#!/usr/bin/env python3

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


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


class CppControllerKeyboardPublisher(Node):
    def __init__(self) -> None:
        super().__init__('cpp_controller_keyboard')
        self.topic = self.declare_parameter('topic', '/trajectory_type').value
        self.pub = self.create_publisher(Int32, self.topic, 10)
        self.mode = "diagonal"
        self.get_logger().info(f"publishing trajectory type on: {self.topic}")

    def publish_trajectory(self, value: int) -> None:
        msg = Int32()
        msg.data = value
        self.pub.publish(msg)
        self.get_logger().info(f"published trajectory type: {value}")

    def toggle_mode(self) -> None:
        self.mode = "orbit" if self.mode == "diagonal" else "diagonal"
        self.get_logger().info(f"q/e/z/c mode: {self.mode}")

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
    print("CPP controller keyboard publisher")
    print("Stop: 0")
    print("Lines: w(+Y), d(+X), s(-Y), a(-X)")
    print("Self rotation: o(CCW), p(CW)")
    print("q/e/z/c mode toggle: press m (diagonal <-> orbit)")
    print("Diagonal mode q/e/z/c: (+Y,-X), (+Y,+X), (-Y,-X), (-Y,+X)")
    print("Orbit mode q/e/z/c: (-X center), reverse(+X), reverse(-X), (+X center)")
    print("Quit: x")


def main() -> None:
    rclpy.init()
    node = CppControllerKeyboardPublisher()
    old_settings = termios.tcgetattr(sys.stdin)
    print_help()

    try:
        tty.setraw(sys.stdin.fileno())
        running = True
        while running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            key = read_key(0.03)
            if not key:
                continue
            k = key.lower()
            if k == 'x':
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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

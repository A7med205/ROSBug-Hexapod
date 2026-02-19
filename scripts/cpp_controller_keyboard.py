#!/usr/bin/env python3

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


KEY_TO_TRAJECTORY = {
    '0': 0,
    '1': 1,
    '2': 2,
    '3': 3,
    '4': 4,
    '5': 5,
    's': 5,
    'S': 5,
}


class CppControllerKeyboardPublisher(Node):
    def __init__(self) -> None:
        super().__init__('cpp_controller_keyboard')
        self.topic = self.declare_parameter('topic', '/trajectory_type').value
        self.pub = self.create_publisher(Int32, self.topic, 10)
        self.get_logger().info(f"publishing trajectory type on: {self.topic}")

    def publish_trajectory(self, value: int) -> None:
        msg = Int32()
        msg.data = value
        self.pub.publish(msg)
        self.get_logger().info(f"published trajectory type: {value}")


def read_key(timeout_sec: float = 0.05) -> str:
    ready, _, _ = select.select([sys.stdin], [], [], timeout_sec)
    return sys.stdin.read(1) if ready else ''


def main() -> None:
    rclpy.init()
    node = CppControllerKeyboardPublisher()
    old_settings = termios.tcgetattr(sys.stdin)

    print('CPP controller keyboard publisher')
    print('Press 0..5 to publish trajectory type (s also sends 5=stationary), q to quit.')

    try:
        tty.setraw(sys.stdin.fileno())
        running = True
        while running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            key = read_key(0.03)
            if not key:
                continue
            if key == 'q':
                running = False
                continue
            if key in KEY_TO_TRAJECTORY:
                node.publish_trajectory(KEY_TO_TRAJECTORY[key])
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

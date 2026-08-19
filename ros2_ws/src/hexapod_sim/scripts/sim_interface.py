#!/usr/bin/env python3
"""Direct-keyboard ROS action adapter for the shared lite gait core."""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path
from typing import Callable

try:
    from common.keyboard_input import KeyboardInput, help_text
    from robot_core import CommandKind, ControllerMode, JointBatch, PostureAxis
    from robot_core.coordinator import HexapodCoordinator
except ImportError:
    for candidate in Path(__file__).resolve().parents:
        if (candidate / "robot_core").is_dir() and (candidate / "common").is_dir():
            sys.path.insert(0, str(candidate))
            break
    from common.keyboard_input import KeyboardInput, help_text
    from robot_core import CommandKind, ControllerMode, JointBatch, PostureAxis
    from robot_core.coordinator import HexapodCoordinator

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


JOINT_NAMES = [
    f"jl{leg}{joint}"
    for leg in range(1, 7)
    for joint in range(1, 4)
]


class SimulationBatchExecutor(Node):
    def __init__(self) -> None:
        super().__init__("hexapod_sim_interface")
        self.action_name = str(
            self.declare_parameter(
                "action_name",
                "/joint_trajectory_controller/follow_joint_trajectory",
            ).value
        )
        self.wait_timeout = float(self.declare_parameter("wait_timeout", 10.0).value)
        self.action_client = ActionClient(self, FollowJointTrajectory, self.action_name)

    def wait_for_server(self) -> bool:
        self.get_logger().info(f"Waiting for action server: {self.action_name}")
        if self.action_client.wait_for_server(timeout_sec=self.wait_timeout):
            return True
        self.get_logger().error(
            f"Action server unavailable after {self.wait_timeout:.1f}s: {self.action_name}"
        )
        return False

    @staticmethod
    def _goal_from_batch(batch: JointBatch):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES
        for point_index, positions in enumerate(batch.points):
            point = JointTrajectoryPoint()
            point.positions = list(positions)
            point.time_from_start = Duration(
                seconds=(point_index + 1) * batch.sample_period
            ).to_msg()
            goal.trajectory.points.append(point)
        return goal

    def _wait_future(self, future, poll_input: Callable[[], None]) -> bool:
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.01)
            poll_input()
        return future.done()

    def execute(self, batch: JointBatch, poll_input: Callable[[], None]) -> bool:
        send_future = self.action_client.send_goal_async(self._goal_from_batch(batch))
        if not self._wait_future(send_future, poll_input):
            return False
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"Goal {batch.goal_id} was rejected")
            return False

        result_future = goal_handle.get_result_async()
        if not self._wait_future(result_future, poll_input):
            return False
        wrapper = result_future.result()
        if wrapper is None:
            self.get_logger().error(f"Goal {batch.goal_id} returned no result")
            return False
        result = wrapper.result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(
                f"Goal {batch.goal_id} failed: {result.error_code} {result.error_string}"
            )
            return False

        deadline = time.monotonic() + batch.hold_after
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            poll_input()
        return rclpy.ok()


def _describe_command(command) -> str:
    if command.kind == CommandKind.WALK:
        if command.steps is not None:
            return f"trajectory {command.trajectory_id} for {command.steps} steps"
        return f"trajectory {command.trajectory_id} continuously"
    if command.kind == CommandKind.TOGGLE_MODE:
        return "cycle normal/auto/posture mode"
    if command.kind == CommandKind.POSTURE:
        if command.posture_axis == PostureAxis.ELEVATION:
            return f"elevation {command.posture_delta * 1000.0:+.1f} mm"
        return (
            f"{command.posture_axis} "
            f"{math.degrees(command.posture_delta):+.1f} deg"
        )
    return command.kind


def _posture_result_notice(coordinator: HexapodCoordinator) -> str:
    result = coordinator.last_posture_result
    if result is None or not result.was_clamped:
        return ""
    if result.axis == PostureAxis.ELEVATION:
        requested = result.requested_delta * 1000.0
        applied = result.applied_delta * 1000.0
        unit = "mm"
    else:
        requested = math.degrees(result.requested_delta)
        applied = math.degrees(result.applied_delta)
        unit = "deg"
    return f"IK limit: requested {requested:+.3f} {unit}, applying {applied:+.3f} {unit}"


def main() -> None:
    rclpy.init(args=None)
    node = SimulationBatchExecutor()
    coordinator = HexapodCoordinator()
    quit_requested = False
    exit_code = 0

    try:
        if not node.wait_for_server():
            raise SystemExit(1)
        with KeyboardInput() as keyboard:
            print(help_text("Hexapod simulation controller"))

            def poll_input() -> None:
                nonlocal quit_requested
                result = keyboard.poll(0.0)
                for notice in result.notices:
                    node.get_logger().info(notice)
                if result.command is not None:
                    accepted = coordinator.request(result.command)
                    description = _describe_command(result.command)
                    node.get_logger().info(
                        f"Command {'accepted' if accepted else 'ignored'}: {description}"
                    )
                    if accepted and result.command.kind == CommandKind.TOGGLE_MODE:
                        node.get_logger().info(
                            f"Mode: {coordinator.mode}; "
                            f"requested mode: {coordinator.requested_mode}"
                        )
                    if accepted and result.command.kind == CommandKind.POSTURE:
                        notice = _posture_result_notice(coordinator)
                        if notice:
                            node.get_logger().info(notice)
                quit_requested = quit_requested or result.quit_requested

            while rclpy.ok() and not quit_requested:
                poll_input()
                batch = coordinator.next_batch()
                if batch is None:
                    rclpy.spin_once(node, timeout_sec=0.02)
                    continue
                node.get_logger().info(
                    f"Goal {batch.goal_id}: {batch.phase_name}, "
                    f"{batch.point_count} points"
                )
                succeeded = node.execute(batch, poll_input)
                coordinator.complete_batch(batch.goal_id, succeeded)
                if not succeeded:
                    exit_code = 1
                    break
                if coordinator.is_stationary:
                    node.get_logger().info(f"Stationary; mode: {coordinator.mode}")
                elif coordinator.mode == ControllerMode.POSTURE and coordinator.is_idle:
                    pose = coordinator.posture.current_pose
                    node.get_logger().info(
                        "Posture hold: "
                        f"z={pose.z * 1000.0:+.2f} mm, "
                        f"pitch={math.degrees(pose.pitch):+.2f} deg, "
                        f"roll={math.degrees(pose.roll):+.2f} deg"
                    )
    except KeyboardInterrupt:
        exit_code = 130
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()

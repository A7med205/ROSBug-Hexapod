#!/usr/bin/env python3
"""Direct-keyboard ROS action adapter for the shared lite gait core."""

from __future__ import annotations

import math
import sys
from pathlib import Path

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

try:
    from .simulation_batch_executor import SimulationBatchExecutor
except ImportError:
    from simulation_batch_executor import SimulationBatchExecutor


def _describe_command(command) -> str:
    if command.kind == CommandKind.WALK:
        if command.steps is not None:
            return f"trajectory {command.trajectory_id} for {command.steps} steps"
        return f"trajectory {command.trajectory_id} continuously"
    if command.kind == CommandKind.TOGGLE_MODE:
        return "cycle normal/auto/posture mode"
    if command.kind == CommandKind.SIT_DOWN:
        return "sit down and restore standup lock"
    if command.kind == CommandKind.POSTURE:
        if command.posture_axis == PostureAxis.ELEVATION:
            return f"objective elevation {command.posture_value * 1000.0:.1f} mm"
        return (
            f"{command.posture_axis} "
            f"{math.degrees(command.posture_value):+.1f} deg"
        )
    if command.kind == CommandKind.RESET_TILT:
        return "reset posture pitch/roll"
    return command.kind


def _posture_result_notice(coordinator: HexapodCoordinator) -> str:
    result = coordinator.last_posture_result
    if result is None or not result.was_clamped:
        return ""
    if result.axis == PostureAxis.ELEVATION:
        requested = result.requested_value * 1000.0
        applied = result.applied_value * 1000.0
        unit = "mm"
    else:
        requested = math.degrees(result.requested_value)
        applied = math.degrees(result.applied_value)
        unit = "deg"
    return f"posture limit: requested {requested:+.3f} {unit}, applying {applied:+.3f} {unit}"


def main() -> None:
    rclpy.init(args=None)
    node = SimulationBatchExecutor()
    coordinator = HexapodCoordinator()
    quit_requested = False
    exit_code = 0

    try:
        node.get_logger().info(f"Waiting for action server: {node.action_name}")
        if not node.wait_for_server():
            node.get_logger().error(
                f"Action server unavailable after {node.wait_timeout:.1f}s: "
                f"{node.action_name}"
            )
            raise SystemExit(1)
        with KeyboardInput() as keyboard:
            print(help_text("Hexapod simulation controller"))

            def poll_input() -> None:
                nonlocal quit_requested
                result = keyboard.poll(0.0)
                for notice in result.notices:
                    node.get_logger().info(notice)
                if result.command is not None:
                    feedback = coordinator.request_with_feedback(result.command)
                    accepted = feedback.accepted
                    description = _describe_command(result.command)
                    node.get_logger().info(
                        f"Command {'accepted' if accepted else 'ignored'}: "
                        f"{description} ({feedback.code})"
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
                result = node.execute(batch, poll_input)
                coordinator.complete_batch(batch.goal_id, result.succeeded)
                if not result.succeeded:
                    node.get_logger().error(
                        f"Goal {batch.goal_id} failed: "
                        f"{result.status} {result.detail}"
                    )
                    exit_code = 1
                    break
                if coordinator.is_stationary:
                    node.get_logger().info(f"Stationary; mode: {coordinator.mode}")
                elif coordinator.mode == ControllerMode.POSTURE and coordinator.is_idle:
                    pose = coordinator.posture.current_pose
                    node.get_logger().info(
                        "Posture hold: "
                        "elevation="
                        f"{coordinator.posture.current_elevation * 1000.0:.2f} mm, "
                        f"pitch={math.degrees(pose.pitch):+.2f} deg, "
                        f"roll={math.degrees(pose.roll):+.2f} deg"
                    )
    except KeyboardInterrupt:
        exit_code = 130
    finally:
        node.close()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()

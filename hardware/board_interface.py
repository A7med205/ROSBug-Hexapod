#!/usr/bin/env python3
"""Keyboard-to-Servo-2040 host adapter for the shared gait core."""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

try:
    from common.keyboard_input import KeyboardInput, help_text
    from robot_core import CommandKind, ControllerMode, JointBatch, PostureAxis
    from robot_core.coordinator import HexapodCoordinator
except ImportError:
    # Support direct execution as well as ``python -m hardware.board_interface``.
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from common.keyboard_input import KeyboardInput, help_text
    from robot_core import CommandKind, ControllerMode, JointBatch, PostureAxis
    from robot_core.coordinator import HexapodCoordinator

try:
    from .batch_executor import BoardBatchExecutor
except ImportError:
    from batch_executor import BoardBatchExecutor


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
            return f"elevation change {command.posture_value * 1000.0:+.1f} mm"
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


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    # The firmware intentionally waits five seconds after board boot before it
    # starts its receiver, so the first acknowledgment needs a longer window.
    parser.add_argument("--response-timeout", type=float, default=8.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    print("Building shared gait templates...")
    coordinator = HexapodCoordinator()
    executor = BoardBatchExecutor(args.port, args.baudrate, args.response_timeout)
    quit_requested = False
    exit_code = 0

    try:
        with KeyboardInput() as keyboard:
            print(help_text("Hexapod hardware controller"))

            def poll_input() -> None:
                nonlocal quit_requested
                result = keyboard.poll(0.0)
                for notice in result.notices:
                    print(notice)
                if result.command is not None:
                    feedback = coordinator.request_with_feedback(result.command)
                    accepted = feedback.accepted
                    description = _describe_command(result.command)
                    print(
                        f"command {'accepted' if accepted else 'ignored'}: "
                        f"{description} ({feedback.code})"
                    )
                    if accepted and result.command.kind == CommandKind.TOGGLE_MODE:
                        print(
                            f"mode: {coordinator.mode}; "
                            f"requested mode: {coordinator.requested_mode}"
                        )
                    if accepted and result.command.kind == CommandKind.POSTURE:
                        notice = _posture_result_notice(coordinator)
                        if notice:
                            print(notice)
                quit_requested = quit_requested or result.quit_requested

            while not quit_requested:
                poll_input()
                batch = coordinator.next_batch()
                if batch is None:
                    time.sleep(0.02)
                    continue
                print(
                    f"goal {batch.goal_id}: {batch.phase_name}, "
                    f"{batch.point_count} points"
                )
                result = executor.execute(batch, poll_input)
                coordinator.complete_batch(batch.goal_id, result.succeeded)
                if not result.succeeded:
                    print(
                        f"goal {batch.goal_id} failed: "
                        f"{result.status} {result.detail}",
                        file=sys.stderr,
                    )
                    exit_code = 1
                    break
                if coordinator.is_stationary:
                    print(f"stationary; mode: {coordinator.mode}")
                elif coordinator.mode == ControllerMode.POSTURE and coordinator.is_idle:
                    pose = coordinator.posture.current_pose
                    print(
                        "posture hold: "
                        "elevation="
                        f"{coordinator.posture.current_elevation * 1000.0:.2f} mm, "
                        f"pitch={math.degrees(pose.pitch):+.2f} deg, "
                        f"roll={math.degrees(pose.roll):+.2f} deg"
                    )
    except KeyboardInterrupt:
        exit_code = 130
    finally:
        executor.close()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()

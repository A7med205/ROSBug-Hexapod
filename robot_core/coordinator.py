"""Top-level arbitration between gait and posture motion generators."""

from __future__ import annotations

from typing import Optional

from gait_core.lite_gait import (
    ControllerState,
    GaitConfig,
    LiteGaitCoordinator,
)
from posture_core import PostureConfig, PostureCoordinator
from robot_core.control import Command, CommandKind, ControllerMode
from robot_core.motion_batch import GoalIdAllocator, JointBatch


class HexapodCoordinator:
    """Own readiness, three operating modes, and exclusive joint output."""

    def __init__(
        self,
        gait_config: Optional[GaitConfig] = None,
        posture_config: Optional[PostureConfig] = None,
    ) -> None:
        goal_ids = GoalIdAllocator()
        self.gait = LiteGaitCoordinator(gait_config, goal_ids)
        self.posture = PostureCoordinator(self.gait.model, posture_config, goal_ids)
        self.mode = ControllerMode.NORMAL
        self.requested_mode = ControllerMode.NORMAL
        self._pending_owner: Optional[str] = None

    @property
    def model(self):
        return self.gait.model

    @property
    def state(self) -> str:
        if self.mode == ControllerMode.POSTURE:
            return self.posture.state
        return self.gait.state

    @property
    def auto_job(self):
        return self.gait.auto_job

    @property
    def current_joint_goal(self):
        if self.mode == ControllerMode.POSTURE:
            return self.posture.current_joint_goal
        return self.gait.current_joint_goal

    @property
    def pending_batch(self) -> Optional[JointBatch]:
        if self._pending_owner == "gait":
            return self.gait.pending_batch
        if self._pending_owner == "posture":
            return self.posture.pending_batch
        return None

    @property
    def is_idle(self) -> bool:
        return self.pending_batch is None and (
            not self.posture.is_busy if self.mode == ControllerMode.POSTURE else True
        )

    @property
    def is_stationary(self) -> bool:
        if self.mode == ControllerMode.POSTURE:
            return self.posture.is_neutral and not self.posture.is_busy
        return self.gait.is_stationary

    @property
    def last_posture_result(self):
        return self.posture.last_plan_result

    def _ready_for_modes(self) -> bool:
        return self.gait.state not in (
            ControllerState.AWAITING_STARTUP,
            ControllerState.STARTING,
            ControllerState.SITTING,
        )

    def _activate_requested_mode_if_ready(self) -> None:
        target = self.requested_mode
        if self.mode == ControllerMode.POSTURE:
            if target == ControllerMode.POSTURE:
                return
            if self.posture.is_neutral and not self.posture.is_busy:
                self.gait.current_joint_goal = list(self.posture.current_joint_goal)
                if not self.gait.request(Command.set_mode(target)):
                    raise RuntimeError(f"could not activate gait mode {target}")
                self.mode = target
            else:
                self.posture.request_return_to_neutral()
            return

        if target == ControllerMode.POSTURE:
            if self.gait.is_stationary:
                self.posture.reset_neutral(self.gait.current_joint_goal)
                self.mode = ControllerMode.POSTURE
            else:
                self.gait.request(Command.stop())
            return

        if self.gait.mode != target:
            if not self.gait.request(Command.set_mode(target)):
                raise RuntimeError(f"could not request gait mode {target}")
        if self.gait.mode == target:
            self.mode = target

    def _request_mode(self, target: str) -> bool:
        if target not in ControllerMode.ORDER or not self._ready_for_modes():
            return False
        self.requested_mode = target

        if self.mode == ControllerMode.POSTURE:
            if target == ControllerMode.POSTURE:
                self.posture.cancel_return()
            else:
                self.posture.request_return_to_neutral()
        elif target == ControllerMode.POSTURE:
            self.gait.request(Command.stop())
        elif target != self.gait.mode:
            self.gait.request(Command.set_mode(target))

        self._activate_requested_mode_if_ready()
        return True

    def request(self, command: Command) -> bool:
        if command.kind == CommandKind.TOGGLE_MODE:
            return self._request_mode(ControllerMode.next(self.requested_mode))
        if command.kind == CommandKind.SET_MODE:
            return command.mode is not None and self._request_mode(command.mode)

        if command.kind in (CommandKind.STARTUP, CommandKind.SKIP_STARTUP):
            if self.mode != ControllerMode.NORMAL or self.requested_mode != self.mode:
                return False
            return self.gait.request(command)

        if command.kind == CommandKind.SIT_DOWN:
            if not self.is_stationary:
                return False
            if self.mode == ControllerMode.POSTURE:
                self.gait.current_joint_goal = list(self.posture.current_joint_goal)
            accepted = self.gait.request(command)
            if accepted:
                self.mode = ControllerMode.NORMAL
                self.requested_mode = ControllerMode.NORMAL
            return accepted

        if command.kind == CommandKind.STOP:
            if self.mode == ControllerMode.POSTURE:
                if self.posture.is_busy:
                    return self.posture.request_interrupt()
                if self.posture.is_neutral:
                    return True
                return self.posture.request_return_to_neutral()
            return self.gait.request(command)

        if command.kind == CommandKind.POSTURE:
            if (
                self.mode != ControllerMode.POSTURE
                or self.requested_mode != ControllerMode.POSTURE
                or command.posture_axis is None
                or command.posture_delta is None
            ):
                return False
            return self.posture.request_delta(
                command.posture_axis,
                command.posture_delta,
            )

        if command.kind == CommandKind.RESET_TILT:
            if (
                self.mode != ControllerMode.POSTURE
                or self.requested_mode != ControllerMode.POSTURE
            ):
                return False
            return self.posture.request_tilt_reset()

        if command.kind == CommandKind.WALK:
            if self.mode not in (ControllerMode.NORMAL, ControllerMode.AUTO):
                return False
            if self.requested_mode != self.mode:
                return False
            return self.gait.request(command)
        return False

    def next_batch(self) -> Optional[JointBatch]:
        if self._pending_owner is not None:
            return None
        self._activate_requested_mode_if_ready()

        if self.mode == ControllerMode.POSTURE:
            batch = self.posture.next_batch()
            if batch is not None:
                self._pending_owner = "posture"
            return batch

        batch = self.gait.next_batch()
        if batch is not None:
            self._pending_owner = "gait"
        return batch

    def complete_batch(self, goal_id: int, succeeded: bool = True) -> None:
        owner = self._pending_owner
        if owner is None:
            raise ValueError(f"goal {goal_id} is not the pending batch")
        self._pending_owner = None
        if owner == "gait":
            self.gait.complete_batch(goal_id, succeeded)
            if succeeded and self.gait.is_stationary:
                self.posture.reset_neutral(self.gait.current_joint_goal)
        elif owner == "posture":
            self.posture.complete_batch(goal_id, succeeded)
        else:
            raise RuntimeError(f"unknown pending owner: {owner}")
        if succeeded:
            self._activate_requested_mode_if_ready()

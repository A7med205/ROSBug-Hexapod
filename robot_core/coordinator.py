"""Top-level arbitration between gait and posture motion generators."""

from __future__ import annotations

from collections import deque
from typing import Deque, Optional

from gait_core.lite_gait import (
    ControllerState,
    GaitConfig,
    LiteGaitCoordinator,
)
from posture_core import PostureConfig, PostureCoordinator
from robot_core.control import Command, CommandKind, ControllerMode, PostureAxis
from robot_core.motion_batch import GoalIdAllocator, JointBatch
from robot_core.feedback import (
    CommandFeedback,
    CoordinatorEvent,
    CoordinatorEventKind,
    CoordinatorStatus,
)


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
        self.mode = ControllerMode.AUTO
        self.requested_mode = ControllerMode.AUTO
        self._pending_owner: Optional[str] = None
        self._next_operation_id = 1
        self._operation_id: Optional[int] = None
        self._operation_command: Optional[Command] = None
        self._events: Deque[CoordinatorEvent] = deque()

    @property
    def model(self):
        return self.gait.model

    def _in_standup_cycle(self) -> bool:
        return self.gait.state in (
            ControllerState.AWAITING_STAND_UP,
            ControllerState.STANDING_UP,
            ControllerState.SITTING_DOWN,
        )

    @property
    def state(self) -> str:
        if self._in_standup_cycle():
            return self.gait.state
        if self.mode == ControllerMode.POSTURE:
            return self.posture.state
        return self.gait.state

    @property
    def auto_job(self):
        return self.gait.auto_job

    @property
    def current_joint_goal(self):
        if self._in_standup_cycle():
            return self.gait.current_joint_goal
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
        if self._in_standup_cycle():
            return False
        if self.mode == ControllerMode.POSTURE:
            return self.posture.is_neutral and not self.posture.is_busy
        return self.gait.is_stationary

    @property
    def last_posture_result(self):
        return self.posture.last_plan_result

    def status(self) -> CoordinatorStatus:
        """Return a transport-neutral snapshot for UIs and remote adapters."""
        pending = self.pending_batch
        auto = self.auto_job
        return CoordinatorStatus(
            state=self.state,
            mode=self.mode,
            requested_mode=self.requested_mode,
            operation_id=self._operation_id,
            command_kind=(
                self._operation_command.kind if self._operation_command else None
            ),
            pending_goal_id=pending.goal_id if pending else None,
            auto_requested_steps=auto.requested_steps if auto else None,
            auto_completed_half_steps=auto.completed_half_steps if auto else None,
            auto_remaining_half_steps=auto.remaining_half_steps if auto else None,
        )

    def drain_events(self):
        """Return and clear structured events accumulated since the last drain."""
        events = tuple(self._events)
        self._events.clear()
        return events

    def _ready_for_modes(self) -> bool:
        return self.gait.state not in (
            ControllerState.AWAITING_STAND_UP,
            ControllerState.STANDING_UP,
            ControllerState.SITTING_DOWN,
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

    def _request(self, command: Command) -> bool:
        if command.kind == CommandKind.TOGGLE_MODE:
            return self._request_mode(ControllerMode.next(self.requested_mode))
        if command.kind == CommandKind.SET_MODE:
            return command.mode is not None and self._request_mode(command.mode)

        if command.kind in (CommandKind.STAND_UP, CommandKind.SKIP_STAND_UP):
            return self.gait.request(command)

        if not self._ready_for_modes():
            return False

        if command.kind == CommandKind.SIT_DOWN:
            if not self.is_stationary:
                return False
            if self.mode == ControllerMode.POSTURE:
                self.gait.current_joint_goal = list(self.posture.current_joint_goal)
            return self.gait.request(command)

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
                or command.posture_value is None
            ):
                return False
            if command.posture_axis == PostureAxis.ELEVATION:
                return self.posture.request_elevation_delta(command.posture_value)
            return self.posture.request_delta(
                command.posture_axis,
                command.posture_value,
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

    def request_with_feedback(self, command: Command) -> CommandFeedback:
        """Request a command and return a structured acceptance result."""
        accepted = self._request(command)
        if not accepted:
            detail = (
                f"{command.kind} rejected in state={self.state}, "
                f"mode={self.mode}, requested_mode={self.requested_mode}"
            )
            feedback = CommandFeedback(False, "command_rejected", detail)
            self._events.append(
                CoordinatorEvent(
                    CoordinatorEventKind.COMMAND_REJECTED,
                    None,
                    command.kind,
                    self.state,
                    self.mode,
                    detail,
                )
            )
            return feedback

        if self._operation_id is not None:
            self._events.append(
                CoordinatorEvent(
                    CoordinatorEventKind.OPERATION_REPLACED,
                    self._operation_id,
                    self._operation_command.kind if self._operation_command else None,
                    self.state,
                    self.mode,
                    f"replaced by {command.kind}",
                )
            )
        operation_id = self._next_operation_id
        self._next_operation_id += 1
        self._operation_id = operation_id
        self._operation_command = command
        detail = (
            f"{command.kind} accepted in state={self.state}, "
            f"mode={self.mode}, requested_mode={self.requested_mode}"
        )
        self._events.append(
            CoordinatorEvent(
                CoordinatorEventKind.COMMAND_ACCEPTED,
                operation_id,
                command.kind,
                self.state,
                self.mode,
                detail,
            )
        )
        self._finish_operation_if_ready()
        return CommandFeedback(True, "accepted", detail, operation_id)

    def request(self, command: Command) -> bool:
        """Backward-compatible boolean command request."""
        return self.request_with_feedback(command).accepted

    def next_batch(self) -> Optional[JointBatch]:
        if self._pending_owner is not None:
            return None
        self._activate_requested_mode_if_ready()

        if self.gait.state in (
            ControllerState.STANDING_UP,
            ControllerState.SITTING_DOWN,
        ):
            batch = self.gait.next_batch()
            if batch is not None:
                self._pending_owner = "gait"
            if batch is None:
                self._finish_operation_if_ready()
            return batch

        if self.mode == ControllerMode.POSTURE:
            batch = self.posture.next_batch()
            if batch is not None:
                self._pending_owner = "posture"
            if batch is None:
                self._finish_operation_if_ready()
            return batch

        batch = self.gait.next_batch()
        if batch is not None:
            self._pending_owner = "gait"
        if batch is None:
            self._finish_operation_if_ready()
        return batch

    def complete_batch(self, goal_id: int, succeeded: bool = True):
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
            event = CoordinatorEvent(
                CoordinatorEventKind.BATCH_COMPLETED,
                self._operation_id,
                self._operation_command.kind if self._operation_command else None,
                self.state,
                self.mode,
                f"goal {goal_id} completed",
            )
            self._events.append(event)
            self._finish_operation_if_ready()
            return event
        event = CoordinatorEvent(
            CoordinatorEventKind.OPERATION_FAILED,
            self._operation_id,
            self._operation_command.kind if self._operation_command else None,
            self.state,
            self.mode,
            f"goal {goal_id} failed",
        )
        self._events.append(event)
        self._operation_id = None
        self._operation_command = None
        return event

    def _operation_has_work(self) -> bool:
        command = self._operation_command
        if command is None or self.pending_batch is not None:
            return self.pending_batch is not None
        if command.kind == CommandKind.WALK:
            return self.auto_job is not None or not self.gait.is_stationary
        if command.kind == CommandKind.STAND_UP:
            return self.gait.state == ControllerState.STANDING_UP
        if command.kind == CommandKind.SIT_DOWN:
            return self.gait.state == ControllerState.SITTING_DOWN
        if command.kind in (CommandKind.POSTURE, CommandKind.RESET_TILT):
            return self.posture.is_busy
        if command.kind in (CommandKind.SET_MODE, CommandKind.TOGGLE_MODE):
            return self.mode != self.requested_mode or not self.is_idle
        if command.kind == CommandKind.STOP:
            return not self.is_stationary
        return False

    def _finish_operation_if_ready(self) -> None:
        if self._operation_id is None or self._operation_has_work():
            return
        self._events.append(
            CoordinatorEvent(
                CoordinatorEventKind.OPERATION_COMPLETED,
                self._operation_id,
                self._operation_command.kind if self._operation_command else None,
                self.state,
                self.mode,
                "operation completed",
            )
        )
        self._operation_id = None
        self._operation_command = None
